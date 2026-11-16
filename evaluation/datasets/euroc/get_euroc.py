#!/usr/bin/env python3
"""Download one EuRoC MAV sequence in ASL format (the folder layout run.sh reads) and optionally copy it to the board.

  get_euroc.py <sequence> [--zip <downloaded zip>] [--out ~/navlib_data/euroc] [--board h15] [--stereo]
  get_euroc.py --list

  sequence: MH_01_easy .. MH_05_difficult, V1_01_easy .. V1_03_difficult, V2_01_easy .. V2_03_difficult

Source: ETH Research Collection, https://doi.org/10.3929/ethz-b-000690084 (EuRoC MAV datasets, CC BY 4.0).
ETH publishes one zip per environment (machine_hall 12.7 GB, vicon_room1 6.0 GB, vicon_room2 6.0 GB; each sequence
inside as a ROS bag + an ASL zip). Without --zip the script reads only the zip's index and the one sequence's ASL
zip with HTTP range requests (~1-2.5 GB). The server may refuse automated downloads (HTTP 429); then download the
environment zip in a browser from the page above and pass it with --zip (the sequence's own ASL zip works too).

Result: <out>/<sequence>/ with cam0/ (images + data.csv), imu0/, state_groundtruth_estimate0/ (ground truth),
leica0/ or vicon0/, and the sensor.yaml files; cam1 only with --stereo. With --board, the folder is copied to
<board>:/home/root/navlib/data/<sequence>, ready for `run.sh euroc_reduced /home/root/navlib/data/<sequence> <out>`.
Python standard library only.
"""
import argparse
import io
import os
import shutil
import subprocess
import sys
import urllib.error
import urllib.request
import zipfile

BASE = 'https://www.research-collection.ethz.ch/server/api/core/bitstreams/{}/content'
ENVS = {  # environment zip on the Research Collection (bitstream id) -> its sequences
    'machine_hall': ('7b2419c1-62b5-4714-b7f8-485e5fe3e5fe',
                     ['MH_01_easy', 'MH_02_easy', 'MH_03_medium', 'MH_04_difficult', 'MH_05_difficult']),
    'vicon_room1': ('02ecda9a-298f-498b-970c-b7c44334d880', ['V1_01_easy', 'V1_02_medium', 'V1_03_difficult']),
    'vicon_room2': ('ea12bc01-3677-4b4c-853d-87c7870b8c44', ['V2_01_easy', 'V2_02_medium', 'V2_03_difficult']),
}
UA = {'User-Agent': 'navlib-get-euroc'}


class HttpFile(io.RawIOBase):
    """Seekable read-only view of a remote file through HTTP range requests (with a read-ahead buffer)."""

    def __init__(self, url, block=1 << 20):
        self.url, self.block, self.pos = url, block, 0
        req = urllib.request.Request(url, headers={**UA, 'Range': 'bytes=0-0'})
        with urllib.request.urlopen(req, timeout=60) as r:
            if r.status != 206:
                raise IOError(f'server does not support range requests ({r.status})')
            self.size = int(r.headers['Content-Range'].split('/')[1])
        self.buf_start, self.buf = 0, b''

    def readable(self):
        return True

    def seekable(self):
        return True

    def tell(self):
        return self.pos

    def seek(self, off, whence=0):
        self.pos = {0: off, 1: self.pos + off, 2: self.size + off}[whence]
        return self.pos

    def _fetch(self, start, n):
        end = min(start + n, self.size) - 1
        for attempt in range(5):
            try:
                req = urllib.request.Request(self.url, headers={**UA, 'Range': f'bytes={start}-{end}'})
                with urllib.request.urlopen(req, timeout=120) as r:
                    return r.read()
            except OSError as e:
                if attempt == 4:
                    raise
                print(f'  retry ({e})', file=sys.stderr)

    def read(self, n=-1):
        if n < 0:
            n = self.size - self.pos
        n = min(n, self.size - self.pos)
        if n <= 0:
            return b''
        if not (self.buf_start <= self.pos and self.pos + n <= self.buf_start + len(self.buf)):
            self.buf_start, self.buf = self.pos, self._fetch(self.pos, max(n, self.block))
        o = self.pos - self.buf_start
        self.pos += n
        return self.buf[o:o + n]

    def readinto(self, b):
        d = self.read(len(b))
        b[:len(d)] = d
        return len(d)


def env_of(seq):
    for env, (bid, seqs) in ENVS.items():
        if seq in seqs:
            return env, bid
    sys.exit(f'unknown sequence {seq}; one of: ' + ' '.join(s for _, ss in ENVS.values() for s in ss))


def copy_member(outer, member, tmp_zip):
    size = outer.getinfo(member).file_size
    with outer.open(member) as src, open(tmp_zip, 'wb') as dst:
        done = 0
        while chunk := src.read(8 << 20):
            dst.write(chunk)
            done += len(chunk)
            print(f'\r  {done / 1e9:.2f} / {size / 1e9:.2f} GB', end='', flush=True)
    print()


def fetch_seq_zip(seq, local_zip, tmp_zip):
    """Put the sequence's ASL zip at tmp_zip; return its path (or local_zip itself if that already is one)."""
    env, bid = env_of(seq)
    if local_zip:
        z = zipfile.ZipFile(local_zip)
        if any(n.startswith('mav0/') for n in z.namelist()):
            return local_zip                                    # the sequence's ASL zip itself
        member = next((n for n in z.namelist() if n.endswith(f'{seq}/{seq}.zip')), None)
        if not member:
            sys.exit(f'{local_zip} has neither mav0/ nor {seq}/{seq}.zip (wrong environment? {seq} is in {env}.zip)')
        print(f'{seq}: extracting from {local_zip}')
        copy_member(z, member, tmp_zip)
        return tmp_zip
    url = BASE.format(bid)
    try:
        outer = zipfile.ZipFile(HttpFile(url, block=8 << 20))
    except urllib.error.HTTPError as e:
        sys.exit(f'download refused by the server (HTTP {e.code}). Download {env}.zip in a browser from\n'
                 f'  https://doi.org/10.3929/ethz-b-000690084  (or directly: {url})\n'
                 f'and run again with --zip <path to {env}.zip>')
    member = next(n for n in outer.namelist() if n.endswith(f'{seq}/{seq}.zip'))
    print(f'{seq}: {outer.getinfo(member).file_size / 1e9:.2f} GB from {env}.zip ({url})')
    copy_member(outer, member, tmp_zip)
    return tmp_zip


def download(seq, out_dir, stereo, local_zip=None):
    env_of(seq)
    dest = os.path.join(out_dir, seq)
    if os.path.exists(os.path.join(dest, 'cam0', 'data.csv')):
        print(f'{dest} already there')
        return dest
    os.makedirs(out_dir, exist_ok=True)
    tmp_zip = fetch_seq_zip(seq, local_zip, os.path.join(out_dir, f'.{seq}.zip.part'))

    tmp_dir = dest + '.part'
    shutil.rmtree(tmp_dir, ignore_errors=True)
    with zipfile.ZipFile(tmp_zip) as z:
        bad = z.testzip()
        if bad:
            sys.exit(f'corrupt download ({bad}); delete {tmp_zip} and run again')
        names = [n for n in z.namelist() if n.startswith('mav0/') and not n.endswith('/')
                 and (stereo or not n.startswith('mav0/cam1/'))]
        for i, n in enumerate(names):
            target = os.path.join(tmp_dir, n[len('mav0/'):])
            os.makedirs(os.path.dirname(target), exist_ok=True)
            with z.open(n) as s, open(target, 'wb') as d:
                shutil.copyfileobj(s, d)
            if i % 500 == 0 or i == len(names) - 1:
                print(f'\r  unpacking {i + 1}/{len(names)}', end='', flush=True)
    print()
    os.rename(tmp_dir, dest)
    if tmp_zip != local_zip:
        os.remove(tmp_zip)
    n_img = sum(1 for _ in open(os.path.join(dest, 'cam0', 'data.csv'))) - 1
    n_imu = sum(1 for _ in open(os.path.join(dest, 'imu0', 'data.csv'))) - 1
    gt = os.path.exists(os.path.join(dest, 'state_groundtruth_estimate0', 'data.csv'))
    print(f'{dest}: {n_img} images, {n_imu} IMU rows, ' +
          ('ground truth in state_groundtruth_estimate0/' if gt else 'NO ground truth'))
    return dest


def to_board(dest, board):
    remote = f'/home/root/navlib/data/{os.path.basename(dest)}'
    if subprocess.run(['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=5', board, 'true'],
                      capture_output=True).returncode:
        print(f'board {board} not reachable; copy later with: {sys.argv[0]} {os.path.basename(dest)} '
              f'--out {os.path.dirname(dest)} --board {board}')
        return
    size = subprocess.run(['du', '-sm', dest], capture_output=True, text=True).stdout.split()[0]
    free = subprocess.run(['ssh', board, "df -m /home/root | awk 'NR==2 {print $4}'"], capture_output=True,
                          text=True).stdout.strip()
    if int(size) + 500 > int(free):
        sys.exit(f'not enough space on {board}: need {size} MB, {free} MB free')
    print(f'copying {size} MB to {board}:{remote}')
    tar = subprocess.Popen(['tar', 'cf', '-', '-C', os.path.dirname(dest), os.path.basename(dest)],
                           stdout=subprocess.PIPE)
    ok = subprocess.run(['ssh', board, f'mkdir -p {os.path.dirname(remote)} && tar xf - -C {os.path.dirname(remote)}'],
                        stdin=tar.stdout).returncode == 0
    tar.stdout.close()
    if tar.wait() or not ok:
        sys.exit('copy to the board failed')
    n = subprocess.run(['ssh', board, f'ls {remote}/cam0/data | wc -l'], capture_output=True, text=True).stdout.strip()
    print(f'{board}:{remote}: {n} images')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('sequence', nargs='?')
    ap.add_argument('--zip', help='already downloaded environment zip (e.g. vicon_room1.zip) or sequence ASL zip')
    ap.add_argument('--out', default=os.path.expanduser('~/navlib_data/euroc'))
    ap.add_argument('--board', help='ssh name of the board to copy the sequence to (e.g. h15)')
    ap.add_argument('--stereo', action='store_true', help='also keep cam1 (not needed for mono)')
    ap.add_argument('--list', action='store_true', help='list the sequences')
    a = ap.parse_args()
    if a.list or not a.sequence:
        for env, (_, seqs) in ENVS.items():
            print(f'{env:13s}', ' '.join(seqs))
        return
    dest = download(a.sequence, a.out, a.stereo, a.zip)
    if a.board:
        to_board(dest, a.board)


if __name__ == '__main__':
    main()
