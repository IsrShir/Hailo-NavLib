// vio_rec: record camera frames (V4L2) + LSM6DSR gyro/accel (IIO buffers) on the H15L for offline VIO.
// Raw output; to_asl.sh (board) or analysis/rec2asl.py (laptop) converts it to EuRoC ASL.
//
// Expects the IIO buffers to be configured by imu_setup.sh first (scan elements, ODR,
// timestamp clock); this program only opens /dev/iio:deviceN and reads.
// File I/O never runs on the capture or IMU paths: frames go through a RAM queue to a
// writer thread (-q frames, dropped+counted if it fills), IMU samples stay in RAM until exit.
//
// Output in <out_dir>:
//   cam/<seq>.pgm            Y plane, optionally box-downscaled by -k
//   cam.csv                  seq,v4l2_ts_ns,dqbuf_mono_ns,flags,filename
//   gyro.csv / accel.csv     ts_ns,x_raw,y_raw,z_raw   (+ "# scale=" header)
//   meta.txt                 formats, flags, scales
#include <dirent.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <condition_variable>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace {

constexpr int kNumBuffers = 10;

volatile sig_atomic_t g_stop = 0;
void on_sig(int) { g_stop = 1; }

int64_t mono_ns() {
  timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return static_cast<int64_t>(t.tv_sec) * 1000000000LL + t.tv_nsec;
}

int xioctl(int fd, unsigned long req, void *arg) {
  int r;
  do r = ioctl(fd, req, arg); while (r == -1 && errno == EINTR);
  return r;
}

using File = std::unique_ptr<FILE, int (*)(FILE *)>;
File open_file(const std::string &path, const char *mode) { return File(fopen(path.c_str(), mode), &fclose); }

// First line of a small sysfs file, without the newline.
bool read_str(const std::string &path, std::string &out) {
  File f = open_file(path, "r");
  char buf[256];
  if (!f || !fgets(buf, sizeof buf, f.get())) return false;
  buf[strcspn(buf, "\n")] = 0;
  out = buf;
  return true;
}

// Owns a file descriptor.
struct Fd {
  int fd = -1;
  Fd() = default;
  explicit Fd(int f) : fd(f) {}
  Fd(const Fd &) = delete;
  Fd &operator=(const Fd &) = delete;
  Fd &operator=(Fd &&o) noexcept {
    if (this != &o) {
      if (fd >= 0) close(fd);
      fd = o.fd;
      o.fd = -1;
    }
    return *this;
  }
  ~Fd() {
    if (fd >= 0) close(fd);
  }
};

// ---------------- IIO ----------------

struct IioChannel {
  std::string name;
  int index = 0, bytes = 0, bits = 0, shift = 0, offset = 0;
  bool is_signed = false, big_endian = false;

  int64_t get(const uint8_t *rec) const {
    uint64_t u = 0;
    for (int i = 0; i < bytes; i++) {
      int b = big_endian ? i : bytes - 1 - i;
      u = (u << 8) | rec[offset + b];
    }
    u >>= shift;
    if (bits < 64) {
      u &= (1ULL << bits) - 1;
      if (is_signed && (u & (1ULL << (bits - 1)))) u |= ~((1ULL << bits) - 1);
    }
    return static_cast<int64_t>(u);
  }
};

struct ImuSample {
  int64_t ts;
  int32_t x, y, z;
};

struct IioDevice {
  std::string kind;  // "gyro" / "accel"
  std::string sysfs, devnode;
  std::vector<IioChannel> ch;
  int scan_bytes = 0;
  int ix[3] = {-1, -1, -1}, its = -1;  // channel slots for x,y,z,timestamp
  double scale = 0;
  Fd fd;
  std::vector<ImuSample> samples;  // kept in RAM, written out at the end

  // Find /sys/bus/iio/devices/iio:deviceN whose name ends with "_<kind>" (lsm6dsr_gyro, ...)
  bool find(const std::string &k) {
    kind = k;
    std::unique_ptr<DIR, int (*)(DIR *)> dir(opendir("/sys/bus/iio/devices"), &closedir);
    if (!dir) return false;
    while (dirent *e = readdir(dir.get())) {
      std::string dn = e->d_name, name;
      if (dn.compare(0, 10, "iio:device")) continue;
      if (!read_str("/sys/bus/iio/devices/" + dn + "/name", name)) continue;
      if (name.size() > kind.size() && name[name.size() - kind.size() - 1] == '_' &&
          !name.compare(name.size() - kind.size(), kind.size(), kind)) {
        sysfs = "/sys/bus/iio/devices/" + dn;
        devnode = "/dev/" + dn;
        return true;
      }
    }
    return false;
  }

  // Parse enabled scan elements and compute the packed record layout (each element is
  // aligned to its own storage size, as the IIO core does).
  bool layout() {
    std::string dp = sysfs + "/scan_elements", v;
    std::unique_ptr<DIR, int (*)(DIR *)> dir(opendir(dp.c_str()), &closedir);
    if (!dir) return false;
    ch.clear();
    while (dirent *e = readdir(dir.get())) {
      std::string en = e->d_name;
      if (en.size() < 4 || en.compare(en.size() - 3, 3, "_en")) continue;
      if (!read_str(dp + "/" + en, v) || atoi(v.c_str()) != 1) continue;
      IioChannel c;
      c.name = en.substr(0, en.size() - 3);
      if (!read_str(dp + "/" + c.name + "_index", v)) continue;
      c.index = atoi(v.c_str());
      if (!read_str(dp + "/" + c.name + "_type", v)) continue;
      char endian[3] = {0}, sign;
      int storage;
      if (sscanf(v.c_str(), "%2[bl]e:%c%d/%d>>%d", endian, &sign, &c.bits, &storage, &c.shift) != 5) continue;
      c.big_endian = endian[0] == 'b';
      c.is_signed = sign == 's';
      c.bytes = storage / 8;
      ch.push_back(c);
    }
    std::sort(ch.begin(), ch.end(), [](const IioChannel &a, const IioChannel &b) { return a.index < b.index; });
    int off = 0, maxb = 1;
    for (size_t i = 0; i < ch.size(); i++) {
      IioChannel &c = ch[i];
      if (off % c.bytes) off += c.bytes - off % c.bytes;
      c.offset = off;
      off += c.bytes;
      maxb = std::max(maxb, c.bytes);
      const std::string &n = c.name;
      if (n == "in_timestamp") its = static_cast<int>(i);
      else if (n.size() > 2 && n[n.size() - 2] == '_' && n.back() >= 'x' && n.back() <= 'z')
        ix[n.back() - 'x'] = static_cast<int>(i);
    }
    if (off % maxb) off += maxb - off % maxb;
    scan_bytes = off;
    if (ix[0] < 0 || ix[1] < 0 || ix[2] < 0 || its < 0) {
      fprintf(stderr, "%s: x/y/z/timestamp scan elements not all enabled (run imu_setup.sh)\n", sysfs.c_str());
      return false;
    }
    if (!read_str(sysfs + (kind == "gyro" ? "/in_anglvel_scale" : "/in_accel_scale"), v)) return false;
    scale = atof(v.c_str());
    return true;
  }

  void push(const uint8_t *rec) {
    samples.push_back({ch[its].get(rec), static_cast<int32_t>(ch[ix[0]].get(rec)),
                       static_cast<int32_t>(ch[ix[1]].get(rec)), static_cast<int32_t>(ch[ix[2]].get(rec))});
  }

  void write(FILE *out) const {
    fprintf(out, "# scale=%.12g\n# ts_ns,x_raw,y_raw,z_raw\n", scale);
    for (const ImuSample &s : samples) fprintf(out, "%lld,%d,%d,%d\n", static_cast<long long>(s.ts), s.x, s.y, s.z);
  }
};

// Reads both IIO buffers until g_stop. Samples stay in RAM so eMMC write stalls can never
// back up the IIO buffers (512 samples = ~2.3 s at 221 Hz).
void imu_loop(IioDevice *d) {  // d[0] gyro, d[1] accel
  pollfd pf[2] = {{d[0].fd.fd, POLLIN, 0}, {d[1].fd.fd, POLLIN, 0}};
  uint8_t buf[4096];
  while (!g_stop) {
    if (poll(pf, 2, 200) <= 0) continue;
    for (int k = 0; k < 2; k++) {
      if (!(pf[k].revents & POLLIN)) continue;
      ssize_t n = read(d[k].fd.fd, buf, sizeof buf - sizeof buf % d[k].scan_bytes);
      if (n <= 0) continue;
      for (ssize_t o = 0; o + d[k].scan_bytes <= n; o += d[k].scan_bytes) d[k].push(buf + o);
    }
  }
}

// ---------------- V4L2 ----------------

class Camera {
 public:
  Fd fd;
  uint32_t w = 0, h = 0, stride = 0;
  int down = 1;
  void *map[kNumBuffers] = {};
  size_t len[kNumBuffers] = {};
  bool streaming = false;

  bool open(const char *dev, uint32_t want_w, uint32_t want_h, int fps, int k) {
    down = k;
    fd = Fd(::open(dev, O_RDWR));
    if (fd.fd < 0) { perror(dev); return false; }
    v4l2_format f{};
    f.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    f.fmt.pix_mp.width = want_w;
    f.fmt.pix_mp.height = want_h;
    f.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_NV12M;
    f.fmt.pix_mp.num_planes = 2;
    if (xioctl(fd.fd, VIDIOC_S_FMT, &f)) { perror("VIDIOC_S_FMT"); return false; }
    w = f.fmt.pix_mp.width;
    h = f.fmt.pix_mp.height;
    stride = f.fmt.pix_mp.plane_fmt[0].bytesperline ? f.fmt.pix_mp.plane_fmt[0].bytesperline : w;
    if (fps > 0) {
      v4l2_streamparm sp{};
      sp.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
      sp.parm.capture.timeperframe.numerator = 1;
      sp.parm.capture.timeperframe.denominator = fps;
      if (xioctl(fd.fd, VIDIOC_S_PARM, &sp)) perror("VIDIOC_S_PARM (fps not set)");
    }
    v4l2_requestbuffers rb{};
    rb.count = kNumBuffers;
    rb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    rb.memory = V4L2_MEMORY_MMAP;
    if (xioctl(fd.fd, VIDIOC_REQBUFS, &rb) || rb.count > kNumBuffers) { perror("VIDIOC_REQBUFS"); return false; }
    for (uint32_t i = 0; i < rb.count; i++) {
      v4l2_plane pl[VIDEO_MAX_PLANES] = {};
      v4l2_buffer b{};
      b.type = rb.type;
      b.memory = V4L2_MEMORY_MMAP;
      b.index = i;
      b.length = VIDEO_MAX_PLANES;
      b.m.planes = pl;
      if (xioctl(fd.fd, VIDIOC_QUERYBUF, &b)) { perror("VIDIOC_QUERYBUF"); return false; }
      len[i] = pl[0].length;  // only the Y plane is mapped
      void *m = mmap(nullptr, pl[0].length, PROT_READ, MAP_SHARED, fd.fd, pl[0].m.mem_offset);
      if (m == MAP_FAILED) { perror("mmap"); return false; }
      map[i] = m;
      if (xioctl(fd.fd, VIDIOC_QBUF, &b)) { perror("VIDIOC_QBUF"); return false; }
    }
    v4l2_buf_type t = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    if (xioctl(fd.fd, VIDIOC_STREAMON, &t)) { perror("VIDIOC_STREAMON"); return false; }
    streaming = true;
    return true;
  }

  void stream_off() {
    if (!streaming) return;
    v4l2_buf_type t = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    xioctl(fd.fd, VIDIOC_STREAMOFF, &t);
    streaming = false;
  }

  ~Camera() {
    stream_off();
    for (int i = 0; i < kNumBuffers; i++)
      if (map[i]) munmap(map[i], len[i]);
  }

  uint32_t out_w() const { return w / down; }
  uint32_t out_h() const { return h / down; }

  // Copy (and optionally box-downscale) the mmap'd Y plane into a packed out_w x out_h buffer.
  // Prefer ISP scaling (-k 1): the mmap is uncached and per-byte reads are very slow.
  void copy_y(const uint8_t *y, uint8_t *dst) const {
    uint32_t ow = out_w(), oh = out_h();
    if (down == 1) {
      for (uint32_t r = 0; r < oh; r++) memcpy(dst + static_cast<size_t>(r) * ow, y + static_cast<size_t>(r) * stride, ow);
      return;
    }
    int n = down * down;
    for (uint32_t r = 0; r < oh; r++)
      for (uint32_t q = 0; q < ow; q++) {
        int s = 0;
        for (int dy = 0; dy < down; dy++) {
          const uint8_t *p = y + static_cast<size_t>(r * down + dy) * stride + q * down;
          for (int dx = 0; dx < down; dx++) s += p[dx];
        }
        dst[static_cast<size_t>(r) * ow + q] = static_cast<uint8_t>((s + n / 2) / n);
      }
  }
};

// Frame queue between the capture loop and the writer thread. The capture loop copies the
// Y plane into a free slot and re-queues the V4L2 buffer at once; the writer thread does the
// (possibly stalling) file I/O. A full queue drops the frame and counts it.
struct Slot {
  std::vector<uint8_t> y;
  uint32_t seq = 0, flags = 0;
  int64_t t_v = 0, t_dq = 0;
};

class FrameQueue {
 public:
  FrameQueue(size_t cap, uint32_t w, uint32_t h, std::string out_dir, File csv)
      : slots_(cap), ow_(w), oh_(h), out_(std::move(out_dir)), csv_(std::move(csv)) {
    for (Slot &s : slots_) s.y.resize(static_cast<size_t>(w) * h);  // all buffers up front: no allocation per frame
    writer_ = std::thread(&FrameQueue::writer_loop, this);
  }

  // Capture side: a free slot to fill, or nullptr if the writer is behind (frame dropped).
  Slot *free_slot() {
    std::lock_guard<std::mutex> l(mu_);
    if (count_ == slots_.size()) {
      dropped_++;
      return nullptr;
    }
    return &slots_[(head_ + count_) % slots_.size()];  // only the capture loop writes free slots
  }

  void commit() {
    std::lock_guard<std::mutex> l(mu_);
    count_++;
    max_count_ = std::max(max_count_, count_);
    cv_.notify_one();
  }

  // Writes out everything queued, then stops the writer thread.
  void finish() {
    {
      std::lock_guard<std::mutex> l(mu_);
      done_ = true;
      cv_.notify_one();
    }
    if (writer_.joinable()) writer_.join();
    csv_.reset();
  }

  ~FrameQueue() { finish(); }

  size_t capacity() const { return slots_.size(); }
  size_t max_depth() const { return max_count_; }
  long dropped() const { return dropped_; }

 private:
  void writer_loop() {
    char fn[64];
    for (;;) {
      Slot *s;
      {
        std::unique_lock<std::mutex> l(mu_);
        cv_.wait(l, [this] { return count_ || done_; });
        if (!count_) break;  // done and drained
        s = &slots_[head_];
      }
      snprintf(fn, sizeof fn, "%08u.pgm", s->seq);
      if (File f = open_file(out_ + "/cam/" + fn, "wb")) {
        fprintf(f.get(), "P5\n%u %u\n255\n", ow_, oh_);
        fwrite(s->y.data(), 1, s->y.size(), f.get());
      }
      fprintf(csv_.get(), "%u,%lld,%lld,0x%x,%s\n", s->seq, static_cast<long long>(s->t_v),
              static_cast<long long>(s->t_dq), s->flags, fn);
      std::lock_guard<std::mutex> l(mu_);
      head_ = (head_ + 1) % slots_.size();
      count_--;
    }
  }

  std::vector<Slot> slots_;
  uint32_t ow_, oh_;
  std::string out_;
  File csv_;
  std::mutex mu_;
  std::condition_variable cv_;
  size_t head_ = 0, count_ = 0, max_count_ = 0;
  long dropped_ = 0;
  bool done_ = false;
  std::thread writer_;
};

void usage(const char *a) {
  fprintf(stderr,
          "usage: %s -o <out_dir> [-s seconds] [-d /dev/video0] [-W 1920 -H 1080] [-f fps] [-k down]\n"
          "          [-q queue_frames] [-n keep_every_nth] [--no-cam] [--no-imu]\n",
          a);
}

const char *ts_type(uint32_t flags) {
  switch (flags & V4L2_BUF_FLAG_TIMESTAMP_MASK) {
    case V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC: return "monotonic";
    case V4L2_BUF_FLAG_TIMESTAMP_COPY: return "copy";
    default: return "unknown";
  }
}

}  // namespace

int main(int argc, char **argv) {
  const char *out = nullptr, *vdev = "/dev/video0";
  uint32_t W = 1920, H = 1080;
  int fps = 20, down = 3, secs = 60, qcap = 90, nth = 1;
  bool use_cam = true, use_imu = true;
  for (int i = 1; i < argc; i++) {
    const char *a = argv[i], *v = i + 1 < argc ? argv[i + 1] : nullptr;
    if (!strcmp(a, "-o") && v) out = argv[++i];
    else if (!strcmp(a, "-s") && v) secs = atoi(argv[++i]);
    else if (!strcmp(a, "-d") && v) vdev = argv[++i];
    else if (!strcmp(a, "-W") && v) W = atoi(argv[++i]);
    else if (!strcmp(a, "-H") && v) H = atoi(argv[++i]);
    else if (!strcmp(a, "-f") && v) fps = atoi(argv[++i]);
    else if (!strcmp(a, "-k") && v) down = atoi(argv[++i]);
    else if (!strcmp(a, "-q") && v) qcap = atoi(argv[++i]);
    else if (!strcmp(a, "-n") && v) nth = atoi(argv[++i]);
    else if (!strcmp(a, "--no-cam")) use_cam = false;
    else if (!strcmp(a, "--no-imu")) use_imu = false;
    else { usage(argv[0]); return 2; }
  }
  if (!out || down < 1 || qcap < 2 || nth < 1) { usage(argv[0]); return 2; }
  const std::string out_dir = out;
  mkdir(out, 0755);
  mkdir((out_dir + "/cam").c_str(), 0755);
  signal(SIGINT, on_sig);
  signal(SIGTERM, on_sig);

  File meta = open_file(out_dir + "/meta.txt", "w");
  if (!meta) { perror((out_dir + "/meta.txt").c_str()); return 1; }

  IioDevice imu[2];
  File imu_out[2] = {File(nullptr, &fclose), File(nullptr, &fclose)};
  std::thread imu_thread;
  if (use_imu) {
    const char *kinds[2] = {"gyro", "accel"};
    for (int k = 0; k < 2; k++) {
      IioDevice &d = imu[k];
      if (!d.find(kinds[k]) || !d.layout()) { fprintf(stderr, "IMU %s not ready\n", kinds[k]); return 1; }
      d.fd = Fd(open(d.devnode.c_str(), O_RDONLY | O_NONBLOCK));
      if (d.fd.fd < 0) { perror(d.devnode.c_str()); return 1; }
      d.samples.reserve(static_cast<size_t>(secs > 0 ? secs : 1) * 250 + 4096);  // > 221 Hz: no regrowth while recording
      imu_out[k] = open_file(out_dir + "/" + kinds[k] + ".csv", "w");
      if (!imu_out[k]) { perror((out_dir + "/" + kinds[k] + ".csv").c_str()); return 1; }
      fprintf(meta.get(), "%s: %s scan_bytes=%d scale=%.12g\n", kinds[k], d.sysfs.c_str(), d.scan_bytes, d.scale);
    }
    imu_thread = std::thread(imu_loop, imu);
  }

  int ret = 0;
  Camera cam;
  std::unique_ptr<FrameQueue> q;
  if (use_cam) {
    File csv(nullptr, &fclose);
    if (!cam.open(vdev, W, H, fps, down)) {
      g_stop = 1;
      ret = 1;
    } else if (!(csv = open_file(out_dir + "/cam.csv", "w"))) {
      perror((out_dir + "/cam.csv").c_str());
      g_stop = 1;
      ret = 1;
    } else {
      fprintf(meta.get(), "cam: %s %ux%u stride=%u down=%d -> %ux%u fps_req=%d queue=%d keep_every=%d\n", vdev, cam.w,
              cam.h, cam.stride, down, cam.out_w(), cam.out_h(), fps, qcap, nth);
      fprintf(csv.get(), "# seq,v4l2_ts_ns,dqbuf_mono_ns,flags,filename\n");
      q = std::make_unique<FrameQueue>(qcap, cam.out_w(), cam.out_h(), out_dir, std::move(csv));
    }
  }
  fflush(meta.get());

  const int64_t t_end = mono_ns() + static_cast<int64_t>(secs) * 1000000000LL;
  long frames = 0;
  uint32_t first_flags = 0;
  while (!g_stop && mono_ns() < t_end) {
    if (!use_cam) { usleep(100000); continue; }
    pollfd pf = {cam.fd.fd, POLLIN, 0};
    if (poll(&pf, 1, 1000) <= 0) continue;
    v4l2_plane pl[VIDEO_MAX_PLANES] = {};
    v4l2_buffer b{};
    b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    b.memory = V4L2_MEMORY_MMAP;
    b.length = VIDEO_MAX_PLANES;
    b.m.planes = pl;
    if (xioctl(cam.fd.fd, VIDIOC_DQBUF, &b)) continue;
    const int64_t t_dq = mono_ns();
    const int64_t t_v = static_cast<int64_t>(b.timestamp.tv_sec) * 1000000000LL + static_cast<int64_t>(b.timestamp.tv_usec) * 1000LL;
    if (!frames) first_flags = b.flags;
    if (b.sequence % nth == 0) {  // -n: keep every nth frame only (e.g. high-res lens calibration)
      if (Slot *s = q->free_slot()) {  // nullptr: writer is behind, drop this frame rather than stall capture
        cam.copy_y(static_cast<const uint8_t *>(cam.map[b.index]), s->y.data());
        s->seq = b.sequence;
        s->flags = b.flags;
        s->t_v = t_v;
        s->t_dq = t_dq;
        q->commit();
      }
    }
    xioctl(cam.fd.fd, VIDIOC_QBUF, &b);
    frames++;
  }
  g_stop = 1;
  if (q) {
    cam.stream_off();
    q->finish();
    fprintf(meta.get(), "cam queue: max depth %zu of %zu, dropped %ld (writer too slow)\n", q->max_depth(), q->capacity(),
            q->dropped());
    fprintf(meta.get(), "cam frames=%ld first_flags=0x%x ts_type=%s ts_src=%s\n", frames, first_flags,
            ts_type(first_flags),
            (first_flags & V4L2_BUF_FLAG_TSTAMP_SRC_MASK) == V4L2_BUF_FLAG_TSTAMP_SRC_SOE ? "start-of-exposure"
                                                                                           : "end-of-frame");
  }
  if (use_imu) {
    imu_thread.join();
    for (int k = 0; k < 2; k++) {
      fprintf(meta.get(), "%s samples=%zu\n", imu[k].kind.c_str(), imu[k].samples.size());
      imu[k].write(imu_out[k].get());
      imu_out[k].reset();
    }
  }
  meta.reset();
  if (File m = open_file(out_dir + "/meta.txt", "r")) {
    int c;
    while ((c = fgetc(m.get())) != EOF) putchar(c);
  }
  return ret;
}
