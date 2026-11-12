# NavLib

Visual-inertial navigation (VIO) on the Hailo-15L: record camera + IMU on the board, calibrate the sensors,
run OpenVINS (mono camera + IMU) on the board, and evaluate the trajectory.

## Quick start: the steps in order

| # | step | command | how often |
|---|---|---|---|
| 0 | prerequisites | Hailo-15L SDK installed; board reachable as `ssh h15` ([below](#prerequisites-and-outputs)) | once |
| 1 | build everything and install it on the board | `./setup.sh` | once per board / after code changes |
| 2 | record on the board | `ssh h15 /home/root/navlib/recorder/record.sh walk3 120 -W 768 -H 432 -k 1` | per recording |
| 3 | run OpenVINS on the board | `ssh h15 /home/root/navlib/openvins/run.sh drone /home/root/navlib/rec/walk3/asl /home/root/navlib/res/walk3` | per run |
| 4 | analyse on the laptop | `scp -r h15:/home/root/navlib/res/walk3 .` then `analysis/eval_loop.py walk3/traj.txt --out walk3.png` (loop error + plot) | per run |
| (5) | keep a copy of the recording | `recorder/fetch.sh walk3` (laptop copy + timing report) | optional |

`setup.sh` runs `openvins/build_ceres.sh`, fetches the OpenVINS submodule, runs `build_h15l.sh` and `recorder/build.sh`,
then installs on the board (`openvins/deploy.sh` + `recorder/deploy.sh`) if it is reachable; Ceres is built only once.
`./setup.sh <name>` for another board SSH name, `./setup.sh --no-deploy` to only build.
Calibration (`calibration/`) is only needed for new hardware or a recalibration; it updates `calibration/hw_setups/`.

## How the parts connect

Every run takes two inputs, **settings** and a **dataset**, and writes its results to an output folder:

```
  SETTINGS: how to run                     DATASET: what to run on

  openvins/config/<settings>.yaml          rec/<name>/asl     recorded on the board by recorder/
    + the calibration it names:              or
  calibration/hw_setups/<setup>/           data/<sequence>    EuRoC, from euroc/get_euroc.py
       │                                         │
       └────────────────────┬────────────────────┘
                            ▼
            openvins/run.sh <settings> <dataset> <out>     (on the board)
                            │
                            ▼
            <out>/traj.txt, stats.csv, timing_wall.txt
                            │
                            ▼
            analysis/  plots and errors (laptop)
```

| folder | what | runs on |
|---|---|---|
| `openvins/config/` | **algorithm settings** (threads, features, clones, gates); each file names the hardware setup it uses | - |
| `openvins/` | `open_vins/` = official OpenVINS (git submodule, unmodified); `runner/vio_replay` = replays a recorded dataset (EuRoC ASL folder) through OpenVINS; scripts to build, deploy and run | laptop / board |
| `recorder/` | `vio_rec` (synchronized camera + IMU recorder, C++) and the scripts around it | board / laptop |
| `calibration/` | `hw_setups/` = **one folder per hardware setup** (camera + IMU) with its calibration results (Kalibr format, not tied to OpenVINS); calibration tools: lens (`calib_fisheye_aprilgrid.py`, `check_aprilgrid.py`), camera-IMU rotation + time offset (`cam_imu_targetcal.py`), IMU noise (`allan_variance.py`); the AprilGrid target (`aprilgrid/`) | laptop |
| `analysis/` | tools for any dataset (our recordings or EuRoC): recording timing report and conversion (`rec2asl.py`), walk evaluation without ground truth (`eval_loop.py`), trajectory plots (`plot_traj.py`, `plot_traj_vs_gt.py`), feature statistics (`plot_frame_stats.py`), time per stage (`timing_report.py`), IMU-only dead reckoning | laptop |
| `euroc/` | everything specific to the public EuRoC dataset: download (`get_euroc.py`), ROS bag conversion (`bag2asl.py`, needs ROS 2; only for EuRoC bags), benchmark scripts (`board_bench*.sh` on the board, `analyze.py` on the laptop: time per stage, fps and ATE against the ground truth shipped with OpenVINS) | laptop / board |

Three things are kept apart on purpose:

- **hardware setup** (`calibration/hw_setups/`): the camera + IMU and their calibration. `h15l_imx678` = IMX678 + kit lens + LSM6DSR on the
  H15L SBC; `euroc` = the EuRoC dataset's camera + IMU. Recalibrating changes only these files.
- **settings** (`openvins/config/`): how OpenVINS runs. `drone.yaml` uses `calibration/hw_setups/h15l_imx678`; `euroc_reduced.yaml`
  and `euroc_default.yaml` share `calibration/hw_setups/euroc`. Variants such as 1 vs 4 threads are overrides at run time
  (`run.sh drone <data> <out> num_opencv_threads=1`), not extra files.
- **datasets**: recordings (walk1, walk2, ...) and EuRoC sequences. Not in git (hundreds of MB each).

## Prerequisites and outputs

Not in the repo:

- the Hailo-15L Yocto SDK (v1.13, at `~/h15l_sdk_v1.13/sdk` or set `H15L_SDK`);
- the board reachable over SSH as `h15`. Direct Ethernet: board 10.0.0.1, laptop 10.0.0.2/24, and in `~/.ssh/config`:
  ```
  Host h15
      HostName 10.0.0.1
      User root
      StrictHostKeyChecking no
      UserKnownHostsFile /dev/null
  ```
  (the board's host key changes on reflash). Through a lab shuttle, add `ProxyJump <user>@<shuttle>`. Another name
  works too: `./setup.sh <name>`, and use `<name>` instead of `h15` in the commands here;
- Python 3 with numpy, opencv-python (>= 4.7, includes aruco) and matplotlib.

On the board everything lives in `/home/root/navlib`: `bin/ lib/ openvins/ calibration/hw_setups/ recorder/ euroc/` (installed by
`setup.sh`), `rec/` (recordings), `data/` (other datasets, e.g. EuRoC), `res/` (run results).

`run.sh` writes the exact settings used (`estimator_config.yaml`), `traj.txt` (IMU pose per frame), `stats.csv`
(per frame: reporting times, wall + CPU processing time, active tracks, MSCKF features used, SLAM landmarks in
state / added / removed), `timing_wall.txt` (time per OpenVINS stage) and `log.txt`.

> Tested (2026-11-12, board): `setup.sh` install of the current layout; `run.sh drone` on walk2 (relative paths) and
> `run.sh euroc_reduced` on V1_01 (copied with `euroc/get_euroc.py --board`) give trajectories byte-identical to the
> earlier runs, V1_01 ATE 0.057 m with `analysis/plot_traj_vs_gt.py` (no ROS).
> Tested (2026-11-10): a fresh `git clone --recursive` + `./setup.sh` on the laptop, Ceres included;
> `setup.sh` installing on the board; `to_asl.sh` (byte-identical to `analysis/rec2asl.py`); `run.sh drone` on walk2
> with the official OpenVINS gives a trajectory byte-identical to the earlier runs (Hailo fork + patches), and the
> same per-frame statistics on all 2638 frames; `vio_rec` (C++) matches the former C version (0 drops, same
> frame/IMU timing, same CPU).

## Recording on the board (steps 2-3)

`record.sh <name> <seconds> [vio_rec options]`; `-W 768 -H 432 -k 1` = 768x432 @ 24 fps (the drone setting).
The recording lands in `/home/root/navlib/rec/<name>/` (raw camera + IMU) and `record.sh` finishes by running
`to_asl.sh`, which adds `<name>/asl/` - the EuRoC folder `run.sh` reads (images are symlinks, nothing copied).
`fetch.sh <name>` copies a recording to `~/navlib_data/<name>/` and prints its timing report (dropped frames,
intervals, IMU rate); not needed for running OpenVINS.

EuRoC sequences: `euroc/get_euroc.py V1_01_easy --board h15` puts the sequence (ASL format, with ground truth) in
`~/navlib_data/euroc/` and copies it to `/home/root/navlib/data/V1_01_easy` on the board; then
`run.sh euroc_reduced /home/root/navlib/data/V1_01_easy <out>`. The ETH server may refuse automated downloads;
the script then prints the link: download the environment zip (e.g. `vicon_room1.zip`, 6 GB) in a browser and run
again with `--zip <file>` (tested with zips of the published layout, not yet with ETH's own file).

- Delayed start (to unplug Ethernet first): see the comment at the top of `record.sh`.
- Hold still for ~5 s at the start (OpenVINS initializes from standstill) and at the end; keep the camera away from
  close blank surfaces (VIO loses all features there).
- About 7.7 MB/s at 768x432 @ 24 fps; check free space on the board first.
- `vio_rec --no-cam` records the IMU only (e.g. gyro scale test, Allan variance); `-n N` keeps every Nth frame
  (high-resolution lens calibration).
- `recorder/imu_selftest.sh` runs the IMU's built-in self-test (keep the board still).

## Evaluating a walk

Our recordings have no ground-truth path, so a walk is judged by points with known positions:

- **Loop error**: start and end standing still on the same spot (best: facing the AprilGrid, which gives the true
  start and end pose); the distance between VIO's start and end positions, in % of the path.
- **Same spot twice**: stop on a marked spot on the way out and again on the way back; the distance between VIO's
  two positions there, and the heading/length of the out and back legs (heading difference = yaw error, length
  difference = scale error).

`analysis/eval_loop.py <res>/traj.txt --asl <recording>/asl --same-spot 32-33.5,67.5-70.5 --out walk.png` prints
both and plots the walk; `--asl` (only `cam0/data.csv` and `imu0/data.csv` are read) adds the still periods found
from the IMU, which also give the stop times for `--same-spot`.

## Calibration

1. **Target**: print `calibration/aprilgrid/aprilgrid_6x4_30mm_A4.pdf` at 100%, measure a tag (30 mm), fix it to
   something rigid.
2. **Lens**: record at 1920x1080 keeping every 6th frame (`-W 1920 -H 1080 -k 1 -n 6`), move the target slowly
   through the whole image including edges and corners, then
   `calibration/calib_fisheye_aprilgrid.py <asl> --out calibration/hw_setups/<setup>/lens.yaml`. Check coverage with
   `calibration/check_aprilgrid.py`.
3. **Camera-IMU rotation + time offset**: target fixed, move the camera energetically in all axes (1280x720, `-n 2`),
   then `calibration/cam_imu_targetcal.py <asl> --lens calibration/hw_setups/<setup>/lens.yaml`.
4. Copy the results into `calibration/hw_setups/<setup>/kalibr_imucam_chain.yaml` (manual for now, see To do).
5. **IMU noise** (Allan variance): `vio_rec --no-cam` for 2-3 h with the board still (not touched, no fan blowing on
   it), then `calibration/allan_variance.py <recording>`: prints the Kalibr noise values and plots the Allan deviation.
   Copy them into `calibration/hw_setups/<setup>/kalibr_imu_chain.yaml`.

OpenCV and Kalibr's AprilGrid: Kalibr tags have a 2-bit black border, ids run from the bottom row, each tag is
rotated 180 deg against OpenCV's corner order, and the corners must be refined to the X-junctions. Without these
the lens fit is off by 6-14 px; `calib_fisheye_aprilgrid.py` handles all four.

## Results so far (2026-11)

**EuRoC V1_01 mono on the H15L** (4x A53 @ 1.1 GHz, ms/frame, 1 thread; ATE vs ground truth):

| | default | reduced (100 features, 7 clones, 25 SLAM) | reduced, 4 threads | reduced + half res |
|---|---|---|---|---|
| ms/frame | 123.8 | 53.1 | 40.7 | 40.6 |
| ATE | 0.056 m | 0.057 m | 0.057 m | 0.066 m |

- Fewer features (200 -> 100) is the largest single saving (-42 ms) with no accuracy loss; SLAM 50 -> 25 saves
  -27 ms; 11 -> 7 clones -24 ms; half resolution -31 ms but the worst accuracy.
- With reduced settings, tracking and the filter take about half each (25 / 28 ms). KLT optical flow is half of
  tracking; image pyramid and histogram equalization are per-pixel work (ISP candidates).
- Against an RPi5 the H15L is ~6.7x slower overall; the filter updates (dense FP64 matrix work) slow down most
  (8-9x), tracking least (4.8x). Offloading tracking alone gives at most ~2x.

**Drone rig** (`drone.yaml`, 4 threads, 768x432 @ 24 fps, handheld office walks):

- Speed: ~44 ms per update frame (~22-23 fps), just short of real time at 24 fps; filter stages (~28 ms) are
  single-threaded, so threads only speed up tracking.
- Accuracy: 4-5% loop error (walk2 has ground truth: same start/end pose from the AprilGrid, 2.01 m over 42 m).
- VIO is still fragile: small input changes (frame subset, 4% on one gyro axis) swing the heading by +-10 deg and
  the loop error between ~1% and ~8%. A gyro turn test shows the gyro scale is fine (0.1%); focal x1.04 and
  `max_clones: 10` made things worse. Judge changes by the spread over several runs, not by one number.
- A MEMS IMU alone drifts metres within seconds (`analysis/imu_only_deadreckon.py`); VIO bridges ~1 s without
  features (e.g. facing a blank door), at a cost.

## Board notes

- Camera: the ISP scales the sensor image (no crop), so a calibration at 1920x1080 transfers exactly to 768x432
  (intrinsics x0.4). V4L2 timestamps are monotonic, end of frame.
- IMU: LSM6DSR via IIO; set to 208 Hz it runs at ~221 Hz (timestamps are correct). The camera frame start is
  marked in the IMU data (hardware sync), usable to check the camera-IMU time offset.
- OpenVINS `track_frequency` must be slightly above the camera rate (25 for 24 fps): the runner skips frames that
  arrive sooner than 1/track_frequency, so 24 drops ~29% of frames, and anything from 13 to 23.9 gives 12 Hz.
- The runner is deterministic: same dataset + settings -> byte-identical trajectory, regardless of thread count.
- The board's clock resets on boot; file dates on the board are meaningless.

## To do

- IMU noise: the Allan values (allan1, 3 h) are measured but not yet in `calibration/hw_setups/h15l_imx678/kalibr_imu_chain.yaml`
  (still the datasheet x10 placeholders). A static bench test leaves out vibration, temperature and motion, so the
  values need inflating; the inflation is not decided yet (two walks are too few to choose it).
- Calibration: lens recalibration with edge coverage, camera-IMU rotation with the new lens, lever arm; then
  tighten the gates
  (`up_*_chi2_multipler: 5` currently rejects almost nothing).
- Calibration tools should write `calibration/hw_setups/<setup>/kalibr_imucam_chain.yaml` directly.
- Real-time: 4 threads reach ~22-23 fps at 24 fps input; the filter updates are the next target.
- `requirements.txt` for the Python tools.
- Statistics: number of successfully triangulated MSCKF tracks is not in `stats.csv` (the official OpenVINS does not
  expose it; `msckf_used` is after the update gate, so it is not the same). How to add it is still open.

## Third-party code

- OpenVINS (GPL-3.0, https://github.com/rpng/open_vins) is a git submodule at `openvins/open_vins`, pinned to the
  official master commit 6948812 (2025-11-30, newer than release v2.7) and used unmodified. Clone NavLib with
  `git clone --recursive`, or run `git submodule update --init` (`setup.sh` does it). The runner reads its per-frame
  statistics through a `VioManager` subclass, so no patches are needed. The runner links OpenVINS, so it is
  GPL-3.0 derived work; everything that depends on OpenVINS lives in `openvins/`.
- Kalibr (BSD-3) is only downloaded by `calibration/aprilgrid/make_a4.sh` to regenerate the target PDF.
- No license has been chosen for NavLib yet (internal repo).
