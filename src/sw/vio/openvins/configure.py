"""
OpenVINS configuration for the drone (sw/vio/openvins/run_euroc.cpp), written from the same constants the simulated camera uses (sw/vio/scene.py) so the two cannot drift apart.

uv run -m sw.vio.openvins.configure <folder> [--gyro-noise 0.03] [--accel-noise 0.15] [--init-imu-thresh 0.5]
The IMU noise densities are not the MPU6050's datasheet figures (about 1e-4 rad/s/sqrt(Hz)): the rotors shake the gyro about 10 deg/s RMS at 21 Hz and the accelerometer about 0.1 g, which at 100 Hz are densities of about 0.025 rad/s/sqrt(Hz) and 0.14 m/s^2/sqrt(Hz) taken as white.
"""

import argparse
import math
from pathlib import Path

import numpy as np

from sw.vio import scene

ESTIMATOR = """%YAML:1.0
verbosity: "WARNING"
use_fej: true
integration: "rk4"
use_stereo: false
max_cameras: 1
calib_cam_extrinsics: false
calib_cam_intrinsics: false
calib_cam_timeoffset: true
calib_imu_intrinsics: false
calib_imu_g_sensitivity: false
max_clones: 11
max_slam: 50
max_slam_in_update: 25
max_msckf_in_update: 40
dt_slam_delay: 1
gravity_mag: 9.80665
feat_rep_msckf: "GLOBAL_3D"
feat_rep_slam: "ANCHORED_MSCKF_INVERSE_DEPTH"
feat_rep_aruco: "ANCHORED_MSCKF_INVERSE_DEPTH"
try_zupt: false
zupt_chi2_multipler: 0
zupt_max_velocity: 0.1
zupt_noise_multiplier: 10
zupt_max_disparity: 0.5
zupt_only_at_beginning: false
init_window_time: 2.0
init_imu_thresh: {init_imu_thresh}
init_max_disparity: 10.0
init_max_features: 50
init_dyn_use: false
init_dyn_mle_opt_calib: false
init_dyn_mle_max_iter: 50
init_dyn_mle_max_time: 0.05
init_dyn_mle_max_threads: 6
init_dyn_num_pose: 6
init_dyn_min_deg: 10.0
init_dyn_inflation_ori: 10
init_dyn_inflation_vel: 100
init_dyn_inflation_bg: 10
init_dyn_inflation_ba: 100
init_dyn_min_rec_cond: 1e-12
init_dyn_bias_g: [0.0, 0.0, 0.0]
init_dyn_bias_a: [0.0, 0.0, 0.0]
record_timing_information: false
record_timing_filepath: "/tmp/traj_timing.txt"
save_total_state: false
filepath_est: "/tmp/ov_estimate.txt"
filepath_std: "/tmp/ov_estimate_std.txt"
filepath_gt: "/tmp/ov_groundtruth.txt"
use_klt: true
num_pts: 200
fast_threshold: 15
grid_x: 5
grid_y: 4
min_px_dist: 10
knn_ratio: 0.70
track_frequency: 40.0
downsample_cameras: false
num_opencv_threads: 4
histogram_method: "HISTOGRAM"
use_aruco: false
num_aruco: 1024
downsize_aruco: true
up_msckf_sigma_px: {pixel_sigma}
up_msckf_chi2_multipler: 1
up_slam_sigma_px: {pixel_sigma}
up_slam_chi2_multipler: 1
up_aruco_sigma_px: 1
up_aruco_chi2_multipler: 1
use_mask: false
relative_config_imu: "kalibr_imu_chain.yaml"
relative_config_imucam: "kalibr_imucam_chain.yaml"
"""

IMU = """%YAML:1.0
imu0:
  T_i_b: [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]
  accelerometer_noise_density: {accel_noise}
  accelerometer_random_walk: 3.0e-3
  gyroscope_noise_density: {gyro_noise}
  gyroscope_random_walk: 2.0e-4
  rostopic: /imu0
  time_offset: 0.0
  update_rate: 100.0
  model: "kalibr"
  Tw: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
  R_IMUtoGYRO: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
  Ta: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
  R_IMUtoACC: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
  Tg: [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
"""

CAMERA = """%YAML:1.0
cam0:
  T_imu_cam:
{transform}
  cam_overlaps: []
  camera_model: pinhole
  distortion_coeffs: [0.0, 0.0, 0.0, 0.0]
  distortion_model: radtan
  intrinsics: [{focal:.3f}, {focal:.3f}, {cx:.3f}, {cy:.3f}]
  resolution: [{width}, {height}]
  rostopic: /cam0/image_raw
  timeshift_cam_imu: 0.0
"""


def write(folder, gyro_noise=0.03, accel_noise=0.15, init_imu_thresh=0.5, pixel_sigma=1.5):
    folder = Path(folder)
    folder.mkdir(parents=True, exist_ok=True)
    transform = np.eye(4)
    transform[:3, :3] = scene.CAMERA_TO_BODY
    transform[:3, 3] = scene.CAMERA_POSITION
    rows = "\n".join(f"    - [{', '.join(f'{v:.6f}' for v in row)}]" for row in transform)
    (folder / "estimator_config.yaml").write_text(ESTIMATOR.format(init_imu_thresh=init_imu_thresh, pixel_sigma=pixel_sigma))
    (folder / "kalibr_imu_chain.yaml").write_text(IMU.format(gyro_noise=gyro_noise, accel_noise=accel_noise))
    (folder / "kalibr_imucam_chain.yaml").write_text(CAMERA.format(transform=rows, focal=scene.FOCAL, cx=scene.CENTRE[0], cy=scene.CENTRE[1], width=scene.WIDTH, height=scene.HEIGHT))
    return folder / "estimator_config.yaml"


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    parser.add_argument("folder")
    parser.add_argument("--gyro-noise", type=float, default=0.03)
    parser.add_argument("--accel-noise", type=float, default=0.15)
    parser.add_argument("--init-imu-thresh", type=float, default=0.5)
    args = parser.parse_args()
    print(write(args.folder, args.gyro_noise, args.accel_noise, args.init_imu_thresh))
