#include <cstdlib>
#include "utils/quat_ops.h"
// Runs OpenVINS (ov_msckf) on a EuRoC-layout dataset (sw/vio/dataset.py) without ROS: IMU samples and camera frames fed in time order through VioManager, and after each frame the estimate written as one CSV line: time (s), body position in the world (m), body-to-world quaternion (w, x, y, z), velocity (m/s), and the estimated camera-to-IMU time offset (s).
// run_euroc <estimator_config.yaml> <dataset folder> <out.csv>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>

#include "core/VioManager.h"
#include "core/VioManagerOptions.h"
#include "state/State.h"
#include "utils/print.h"
#include "utils/sensor_data.h"

static std::vector<std::vector<std::string>> rows(const std::string &path) {
  std::vector<std::vector<std::string>> out;
  std::ifstream file(path);
  std::string line;
  while (std::getline(file, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::vector<std::string> fields;
    std::stringstream stream(line);
    std::string field;
    while (std::getline(stream, field, ',')) {
      while (!field.empty() && (field.back() == '\r' || field.back() == ' ')) field.pop_back();
      fields.push_back(field);
    }
    out.push_back(fields);
  }
  return out;
}

int main(int argc, char **argv) {
  if (argc != 4) {
    std::cerr << "run_euroc <estimator_config.yaml> <dataset> <out.csv>\n";
    return 2;
  }
  auto parser = std::make_shared<ov_core::YamlParser>(argv[1]);
  std::string verbosity = "WARNING";
  parser->parse_config("verbosity", verbosity);
  ov_core::Printer::setPrintLevel(verbosity);
  ov_msckf::VioManagerOptions params;
  params.print_and_load(parser);
  params.use_multi_threading_subs = false;
  auto vio = std::make_shared<ov_msckf::VioManager>(params);

  std::string root = std::string(argv[2]) + "/mav0/";
  auto imu = rows(root + "imu0/data.csv");
  auto cam = rows(root + "cam0/data.csv");
  std::ofstream out(argv[3]);
  out.precision(9);
  size_t i = 0;
  for (const auto &frame : cam) {
    double time = std::stod(frame[0]) * 1e-9;
    // Every IMU sample up to IMU_LEAD_S past the frame first: OpenVINS interpolates the IMU to the frame's time (and the camera-IMU offset), so it needs readings after it; fed only up to the frame, initialization never finished ("IMU was only propagated 0.947 of 0.960") and updates lost their features.
    const double IMU_LEAD_S = 0.05;
    for (; i < imu.size() && std::stod(imu[i][0]) * 1e-9 <= time + IMU_LEAD_S; i++) {
      ov_core::ImuData sample;
      sample.timestamp = std::stod(imu[i][0]) * 1e-9;
      sample.wm << std::stod(imu[i][1]), std::stod(imu[i][2]), std::stod(imu[i][3]);
      sample.am << std::stod(imu[i][4]), std::stod(imu[i][5]), std::stod(imu[i][6]);
      vio->feed_measurement_imu(sample);
    }
    // OV_GT_INIT: start from the ground truth (simulated datasets) instead of OpenVINS's initializer, to tell the filter's problems from the initializer's.
    static auto truth = std::getenv("OV_GT_INIT") ? rows(root + "state_groundtruth_estimate0/data.csv") : std::vector<std::vector<std::string>>();
    if (!truth.empty() && !vio->initialized() && time > std::stod(truth[0][0]) * 1e-9 + 1.0) {
      size_t k = 0;
      while (k + 1 < truth.size() && std::stod(truth[k + 1][0]) * 1e-9 <= time) k++;
      const auto &row = truth[k];
      Eigen::Quaterniond body(std::stod(row[4]), std::stod(row[5]), std::stod(row[6]), std::stod(row[7]));
      Eigen::Matrix<double, 17, 1> imustate = Eigen::Matrix<double, 17, 1>::Zero();
      imustate(0) = std::stod(row[0]) * 1e-9;
      imustate.block(1, 0, 4, 1) = ov_core::rot_2_quat(body.toRotationMatrix().transpose());
      for (int j = 0; j < 3; j++) {
        imustate(5 + j) = std::stod(row[1 + j]);
        imustate(8 + j) = std::stod(row[8 + j]);
      }
      vio->initialize_with_gt(imustate);
    }
    ov_core::CameraData message;
    message.timestamp = time;
    message.sensor_ids.push_back(0);
    cv::Mat image = cv::imread(root + "cam0/data/" + frame[1], cv::IMREAD_GRAYSCALE);
    if (image.empty()) {
      std::cerr << "unreadable frame " << frame[1] << std::endl;
      continue;
    }
    message.images.push_back(image);
    message.masks.push_back(cv::Mat::zeros(message.images[0].rows, message.images[0].cols, CV_8UC1));
    vio->feed_measurement_camera(message);
    static int frames = 0;
    if (++frames % 100 == 0) {
      // Points with a 3D position (the frame's active tracks) and the points the last MSCKF update used, with the tracker's own picture of its tracks.
      double active_time;
      std::unordered_map<size_t, Eigen::Vector3d> positions;
      std::unordered_map<size_t, Eigen::Vector3d> uvd;
      vio->get_active_tracks(active_time, positions, uvd);
      std::cerr << frames << " frames, t " << time << (vio->initialized() ? " initialized" : " waiting") << ", active tracks " << positions.size()
                << ", MSCKF used " << vio->get_good_features_MSCKF().size() << ", SLAM " << vio->get_features_SLAM().size() << std::endl;
      if (const char *dump = std::getenv("OV_TRACKS")) cv::imwrite(std::string(dump) + "/tracks_" + std::to_string(frames) + ".png", vio->get_historical_viz_image());
    }
    if (!vio->initialized()) continue;
    auto state = vio->get_state();
    // OpenVINS keeps the JPL quaternion of global-to-IMU; written as the body-to-world rotation.
    Eigen::Matrix3d world = state->_imu->Rot().transpose();
    Eigen::Quaterniond q(world);
    Eigen::Vector3d p = state->_imu->pos(), v = state->_imu->vel();
    out << state->_timestamp << ',' << p.x() << ',' << p.y() << ',' << p.z() << ',' << q.w() << ',' << q.x() << ',' << q.y() << ',' << q.z() << ','
        << v.x() << ',' << v.y() << ',' << v.z() << ',' << state->_calib_dt_CAMtoIMU->value()(0) << '\n';
  }
  return 0;
}
