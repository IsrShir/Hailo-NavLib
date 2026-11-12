// ROS-free OpenVINS replay of an EuRoC ASL folder (cam0 + imu0), mirroring ros2_serial_msckf:
// IMU and camera are merged by timestamp; camera frames are processed once IMU passes them.
// usage: vio_replay <estimator_config.yaml> <asl_dir> <out_traj_tum.txt> [out_stats.csv]
// The optional 4th argument writes one line per processed frame: reporting times, processing wall/CPU time and
// feature statistics. Works with unmodified OpenVINS: the statistics are read through a VioManager subclass.
#include <time.h>

#include <algorithm>
#include <chrono>
#include <deque>
#include <fstream>
#include <iomanip>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>

#include "core/VioManager.h"
#include "core/VioManagerOptions.h"
#include "state/State.h"
#include "types/IMU.h"
#include "utils/print.h"
#include "utils/sensor_data.h"

using namespace ov_msckf;

struct Event {
  int64_t ns;
  bool is_cam;
  ov_core::ImuData imu;
  std::string img_path;
};

// VioManager plus read access to what the per-frame statistics need, using only members OpenVINS gives its
// subclasses (no change to OpenVINS itself).
class StatsVioManager : public VioManager {
public:
  using VioManager::VioManager;
  // features tracked in the last image of camera cam
  int active_tracks(size_t cam) {
    auto ids = trackFEATS->get_last_ids();
    auto it = ids.find(cam);
    return it == ids.end() ? 0 : (int)it->second.size();
  }
  // the last camera message was handled by a zero-velocity update only
  bool zupt() const { return did_zupt_update; }
};

static std::set<size_t> slam_ids(const std::shared_ptr<State> &st) {
  std::set<size_t> ids;
  for (const auto &f : st->_features_SLAM)
    ids.insert(f.first);
  return ids;
}

static double cpu_s() {  // CPU time of the whole process (all OpenVINS threads)
  timespec t;
  clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &t);
  return t.tv_sec + 1e-9 * t.tv_nsec;
}

static std::vector<std::vector<std::string>> read_csv(const std::string &path) {
  std::vector<std::vector<std::string>> rows;
  std::ifstream f(path);
  std::string line;
  while (std::getline(f, line)) {
    if (line.empty() || line[0] == '#')
      continue;
    std::vector<std::string> cols;
    std::stringstream ss(line);
    std::string c;
    while (std::getline(ss, c, ','))
      cols.push_back(c);
    rows.push_back(cols);
  }
  return rows;
}

int main(int argc, char **argv) {
  if (argc < 4) {
    printf("usage: %s <estimator_config.yaml> <asl_dir> <out_traj_tum.txt> [out_stats.csv]\n", argv[0]);
    return EXIT_FAILURE;
  }
  std::string config_path = argv[1], asl = argv[2];

  auto parser = std::make_shared<ov_core::YamlParser>(config_path);
  std::string verbosity = "INFO";
  parser->parse_config("verbosity", verbosity);
  ov_core::Printer::setPrintLevel(verbosity);
  VioManagerOptions params;
  params.print_and_load(parser);
  params.use_multi_threading_subs = false;
  if (!parser->successful()) {
    PRINT_ERROR(RED "unable to parse all parameters\n" RESET);
    return EXIT_FAILURE;
  }
  auto sys = std::make_shared<StatsVioManager>(params);

  // Load and merge events (stable sort keeps IMU before camera on equal stamps)
  std::vector<Event> events;
  for (auto &r : read_csv(asl + "/imu0/data.csv")) {
    Event e{std::stoll(r[0]), false, {}, ""};
    e.imu.timestamp = e.ns * 1e-9;
    e.imu.wm << std::stod(r[1]), std::stod(r[2]), std::stod(r[3]);
    e.imu.am << std::stod(r[4]), std::stod(r[5]), std::stod(r[6]);
    events.push_back(e);
  }
  int64_t last_cam_ns = -1;
  for (auto &r : read_csv(asl + "/cam0/data.csv")) {
    events.push_back({std::stoll(r[0]), true, {}, asl + "/cam0/data/" + r[1]});
    last_cam_ns = std::max(last_cam_ns, (int64_t)std::stoll(r[0]));
  }
  std::stable_sort(events.begin(), events.end(), [](const Event &a, const Event &b) { return a.ns < b.ns; });

  std::ofstream traj(argv[3]);
  traj << "# timestamp(s) px py pz qx qy qz qw (JPL quat, as in ov state dump)\n";
  traj << std::fixed;
  traj.precision(9);

  std::ofstream stats;
  if (argc > 4) {
    stats.open(argv[4]);
    stats << "# cam_ts_s,report_wall_s,report_unix_s,proc_ms,cpu_ms,initialized,zupt,updated,pose_reported,active_tracks,"
             "msckf_used,slam_in_state,slam_added,slam_removed,clones\n";
    stats << std::fixed;
  }
  auto run_t0 = std::chrono::steady_clock::now();

  std::deque<ov_core::CameraData> queue;
  double last_cam_ts = -1;
  double load_s = 0, vio_s = 0;
  int frames = 0;
  for (auto &e : events) {
    if (e.ns > last_cam_ns)
      break;
    if (e.is_cam) {
      double ts = e.ns * 1e-9;
      if (last_cam_ts >= 0 && ts < last_cam_ts + 1.0 / params.track_frequency)
        continue;
      last_cam_ts = ts;
      auto t0 = std::chrono::steady_clock::now();
      cv::Mat img = cv::imread(e.img_path, cv::IMREAD_GRAYSCALE);
      load_s += std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
      ov_core::CameraData cam;
      cam.timestamp = ts;
      cam.sensor_ids.push_back(0);
      cam.images.push_back(img);
      cam.masks.push_back(params.use_mask ? params.masks.at(0) : cv::Mat::zeros(img.rows, img.cols, CV_8UC1));
      queue.push_back(cam);
      continue;
    }
    sys->feed_measurement_imu(e.imu);
    double cutoff = e.imu.timestamp - sys->get_state()->_calib_dt_CAMtoIMU->value()(0);
    while (!queue.empty() && queue.front().timestamp < cutoff) {
      std::set<size_t> slam_before = stats.is_open() ? slam_ids(sys->get_state()) : std::set<size_t>();
      double c0 = cpu_s();
      auto t0 = std::chrono::steady_clock::now();
      sys->feed_measurement_camera(queue.front());
      auto t1 = std::chrono::steady_clock::now();
      double c1 = cpu_s();
      vio_s += std::chrono::duration<double>(t1 - t0).count();
      frames++;
      double cam_ts = queue.front().timestamp;
      queue.pop_front();
      bool reported = sys->initialized();
      if (stats.is_open()) {
        auto st = sys->get_state();
        bool zupt = sys->zupt() && st->_timestamp == cam_ts;
        bool updated = sys->initialized() && !zupt && st->_timestamp == cam_ts;  // propagate + feature update ran
        std::set<size_t> slam_after = slam_ids(st);
        int added = 0, removed = 0;
        for (size_t id : slam_after)
          added += !slam_before.count(id);
        for (size_t id : slam_before)
          removed += !slam_after.count(id);
        double unix_s = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
        stats << std::setprecision(6) << cam_ts << "," << std::chrono::duration<double>(t1 - run_t0).count() << "," << unix_s << ","
              << std::setprecision(3) << 1000 * std::chrono::duration<double>(t1 - t0).count() << "," << 1000 * (c1 - c0) << ","
              << (int)sys->initialized() << "," << (int)zupt << "," << (int)updated << "," << (int)reported << ","
              << sys->active_tracks(0) << "," << (updated ? (int)sys->get_good_features_MSCKF().size() : 0) << ","
              << slam_after.size() << "," << added << "," << removed << "," << (int)st->_clones_IMU.size() << "\n";
      }
      if (reported) {
        auto st = sys->get_state();
        Eigen::Vector3d p = st->_imu->pos();
        Eigen::Vector4d q = st->_imu->quat();
        traj << st->_timestamp << " " << p(0) << " " << p(1) << " " << p(2) << " " << q(0) << " " << q(1) << " " << q(2) << " "
             << q(3) << "\n";
      }
    }
  }
  printf("RESULT frames=%d vio_ms_per_frame=%.2f img_load_ms_per_frame=%.2f\n", frames, 1000 * vio_s / frames, 1000 * load_s / frames);
  return EXIT_SUCCESS;
}
