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
    // Every IMU sample up to the frame first.
    for (; i < imu.size() && std::stod(imu[i][0]) * 1e-9 <= time; i++) {
      ov_core::ImuData sample;
      sample.timestamp = std::stod(imu[i][0]) * 1e-9;
      sample.wm << std::stod(imu[i][1]), std::stod(imu[i][2]), std::stod(imu[i][3]);
      sample.am << std::stod(imu[i][4]), std::stod(imu[i][5]), std::stod(imu[i][6]);
      vio->feed_measurement_imu(sample);
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
    if (++frames % 100 == 0) std::cerr << frames << " frames, t " << time << (vio->initialized() ? " initialized" : " waiting") << std::endl;
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
