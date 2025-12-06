#include <fmt/core.h>

#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <vector>

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/multithread/commandgener.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"

using namespace std::chrono;

const std::string keys =
  "{help h usage ? |      | 输出命令行参数说明}"
  "{@config-path   | configs/standard3.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;
  tools::Plotter plotter;
  tools::Recorder recorder;

  io::CBoard cboard(config_path);
  io::Camera camera(config_path);

  auto_aim::YOLO detector(config_path, false);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;

  auto mode = io::Mode::idle;
  auto last_mode = io::Mode::idle;
  int frame_count = 0;
  io::Command last_command;

  while (!exiter.exit()) {
    camera.read(img, t);
    q = cboard.imu_at(t - 1ms);
    mode = cboard.mode;

    if (last_mode != mode) {
      tools::logger()->info("Switch to {}", io::MODES[mode]);
      last_mode = mode;
    }

    // recorder.record(img, q, t);

    solver.set_R_gimbal2world(q);

    auto yolo_start = std::chrono::steady_clock::now();
    auto armors = detector.detect(img);
    auto tracker_start = std::chrono::steady_clock::now();
    auto targets = tracker.track(armors, t);
    auto aimer_start = std::chrono::steady_clock::now();
    auto command = aimer.aim(targets, t, cboard.bullet_speed);
    auto finish = std::chrono::steady_clock::now();

    if (!targets.empty() && aimer.debug_aim_point.valid &&
        std::abs(command.yaw - last_command.yaw) * 57.3 < 2) {
      command.shoot = true;
    }

    if (command.control) {
      last_command = command;
    }

    tools::logger()->info(
      "[{}] yolo: {:.1f}ms, tracker: {:.1f}ms, aimer: {:.1f}ms", frame_count,
      tools::delta_time(tracker_start, yolo_start) * 1e3,
      tools::delta_time(aimer_start, tracker_start) * 1e3,
      tools::delta_time(finish, aimer_start) * 1e3);

    Eigen::Quaterniond gimbal_q = q;
    Eigen::Vector3d ypr = tools::eulers(gimbal_q.toRotationMatrix(), 2, 1, 0);
    auto yaw = ypr[0];

    tools::draw_text(
      img,
      fmt::format("command is {},{:.2f},{:.2f},shoot:{}", command.control,
                  command.yaw * 57.3, command.pitch * 57.3, command.shoot),
      {10, 60}, {154, 50, 205});
    tools::draw_text(img, fmt::format("gimbal yaw{:.2f}", yaw * 57.3), {10, 90}, {255, 255, 255});

    nlohmann::json data;
    data["armor_num"] = armors.size();
    if (!armors.empty()) {
      const auto & armor = armors.front();
      data["armor_x"] = armor.xyz_in_world[0];
      data["armor_y"] = armor.xyz_in_world[1];
      data["armor_yaw"] = armor.ypr_in_world[0] * 57.3;
      data["armor_yaw_raw"] = armor.yaw_raw * 57.3;
      data["armor_center_x"] = armor.center_norm.x;
      data["armor_center_y"] = armor.center_norm.y;
    }

    data["gimbal_yaw"] = yaw * 57.3;
    data["cmd_yaw"] = command.yaw * 57.3;
    data["shoot"] = command.shoot;

    if (!targets.empty()) {
      auto target = targets.front();
      std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
      for (const Eigen::Vector4d & xyza : armor_xyza_list) {
        auto image_points = solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
        tools::draw_points(img, image_points, {0, 255, 0});
      }

      auto aim_point = aimer.debug_aim_point;
      Eigen::Vector4d aim_xyza = aim_point.xyza;
      auto image_points = solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
      if (aim_point.valid) {
        tools::draw_points(img, image_points, {0, 0, 255});
      }

      Eigen::VectorXd x = target.ekf_x();
      data["x"] = x[0];
      data["vx"] = x[1];
      data["y"] = x[2];
      data["vy"] = x[3];
      data["z"] = x[4];
      data["vz"] = x[5];
      data["a"] = x[6] * 57.3;
      data["w"] = x[7];
      data["r"] = x[8];
      data["l"] = x[9];
      data["h"] = x[10];
      data["last_id"] = target.last_id;

      auto ekf = target.ekf();
      data["residual_yaw"] = ekf.data.at("residual_yaw");
      data["residual_pitch"] = ekf.data.at("residual_pitch");
      data["residual_distance"] = ekf.data.at("residual_distance");
      data["residual_angle"] = ekf.data.at("residual_angle");
      data["nis"] = ekf.data.at("nis");
      data["nees"] = ekf.data.at("nees");
      data["nis_fail"] = ekf.data.at("nis_fail");
      data["nees_fail"] = ekf.data.at("nees_fail");
      data["recent_nis_failures"] = ekf.data.at("recent_nis_failures");
    }

    plotter.plot(data);

    cv::resize(img, img, {}, 0.5, 0.5);
    cv::imshow("reprojection", img);
    auto key = cv::waitKey(1);
    if (key == 'q') break;

    cboard.send(command);
    frame_count++;
  }

  return 0;
}