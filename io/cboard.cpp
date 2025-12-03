#include "cboard.hpp"

#include "Eigen/src/Core/AssignEvaluator.h"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"
#include <cstdint>
#include <iostream>

namespace io {
CBoard::CBoard(const std::string& config_path):
    mode(Mode::idle),
    shoot_mode(ShootMode::left_shoot),
    bullet_speed(0),
    queue_(5000)
// 注意: callback的运行会早于Cboard构造函数的完成
{
    tools::logger()->info("[Cboard] Waiting for q...");
    queue_.pop(data_ahead_);
    queue_.pop(data_behind_);
    tools::logger()->info("[Cboard] Opened.");

    this->serial_ = serial_phoenix::Serial();
    this->serial_.open();

    this->read_buffer_.resize(32);
    this->write_buffer_.resize(32);

    std::thread Link_thread([this] {
        while (true) {
            this->serial_.read(this->read_buffer_);
            // 解析数据
            int type = this->read_buffer_[1];
            std::vector<uint8_t> buffer(29);
            std::memcpy(buffer.data(), this->read_buffer_.data() + 2, 29);
            if(type == 0xA0){
                this->read_fun_1(*(Message_phoenix*)this->read_buffer_.data());
            }
            else{
                std::cout << "Unknown message type: " << type << std::endl;
            }
        }
    });
    Link_thread.detach();
}

Eigen::Quaterniond CBoard::imu_at(std::chrono::steady_clock::time_point timestamp) {
    if (data_behind_.timestamp < timestamp)
        data_ahead_ = data_behind_;

    while (true) {
        queue_.pop(data_behind_);
        if (data_behind_.timestamp > timestamp)
            break;
        data_ahead_ = data_behind_;
    }

    Eigen::Quaterniond q_a             = data_ahead_.q.normalized();
    Eigen::Quaterniond q_b             = data_behind_.q.normalized();
    auto t_a                           = data_ahead_.timestamp;
    auto t_b                           = data_behind_.timestamp;
    auto t_c                           = timestamp;
    std::chrono::duration<double> t_ab = t_b - t_a;
    std::chrono::duration<double> t_ac = t_c - t_a;

    // 四元数插值
    auto k                 = t_ac / t_ab;
    Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();

    return q_c;
}

void CBoard::send(Command command) {
    std::memcpy(this->write_buffer_.data(), &command.control, sizeof(bool));
    std::memcpy(this->write_buffer_.data() + 1, &command.yaw, sizeof(float));
    std::memcpy(this->write_buffer_.data() + 5, &command.pitch, sizeof(float));

    this->serial_.write(this->write_buffer_);
}

void CBoard::callback(const can_frame& frame) {
    // auto timestamp = std::chrono::steady_clock::now();

    // if (frame.can_id == quaternion_canid_) {
    //     // 新协议: 原来的四元数被替换为两个角度 (yaw, pitch)，分别位于字节 0~1 和
    //     // 2~3， 使用 int16 / 1e4 表示（单位: rad）。将 yaw/pitch
    //     // 转换为四元数并入队。
    //     double yaw   = (int16_t)((frame.data[0] << 8) | frame.data[1]) / 1e4;
    //     double pitch = (int16_t)((frame.data[2] << 8) | frame.data[3]) / 1e4;

    //     // 假定无 roll，按先 yaw(绕 Z) 再 pitch(绕 Y) 的顺序构造旋转。
    //     Eigen::AngleAxisd yaw_aa(yaw, Eigen::Vector3d::UnitZ());
    //     Eigen::AngleAxisd pitch_aa(pitch, Eigen::Vector3d::UnitY());
    //     Eigen::Quaterniond q = (yaw_aa * pitch_aa).normalized();

    //     queue_.push({ q, timestamp });
    // }

    // else if (frame.can_id == bullet_speed_canid_)
    // {
    //     bullet_speed = (int16_t)(frame.data[0] << 8 | frame.data[1]) / 1e2;
    //     mode         = Mode(frame.data[2]);
    //     shoot_mode   = ShootMode(frame.data[3]);
    //     // 新协议: ft_angle 使用原始 int16 表示，单位为 rad（无 1e4 缩放）
    //     ft_angle = (int16_t)((frame.data[4] << 8) | frame.data[5]);

    //     // 限制日志输出频率为1Hz
    //     static auto last_log_time = std::chrono::steady_clock::time_point::min();
    //     auto now                  = std::chrono::steady_clock::now();

    //     if (bullet_speed > 0 && tools::delta_time(now, last_log_time) >= 1.0) {
    //         tools::logger()->info(
    //             "[CBoard] Bullet speed: {:.2f} m/s, Mode: {}, "
    //             "Shoot mode: {}, FT angle: {:.2f} rad",
    //             bullet_speed,
    //             MODES[mode],
    //             SHOOT_MODES[shoot_mode],
    //             ft_angle
    //         );
    //         last_log_time = now;
    //     }
    // }
}

// 实现方式有待改进
std::string CBoard::read_yaml(const std::string& config_path) {
    auto yaml = tools::load(config_path);

    quaternion_canid_   = tools::read<int>(yaml, "quaternion_canid");
    bullet_speed_canid_ = tools::read<int>(yaml, "bullet_speed_canid");
    send_canid_         = tools::read<int>(yaml, "send_canid");

    if (!yaml["can_interface"]) {
        throw std::runtime_error("Missing 'can_interface' in YAML configuration.");
    }

    return yaml["can_interface"].as<std::string>();
}

void CBoard::read_fun_1(Message_phoenix& msg) {
    auto timestamp = std::chrono::steady_clock::now();

    GimbalControl data = reinterpret_cast<GimbalControl&>(msg);
    float yaw = data.yaw;
    float pitch = data.pitch;
    Eigen::AngleAxisd yaw_aa(yaw, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd pitch_aa(pitch, Eigen::Vector3d::UnitY());
    Eigen::Quaterniond q = (yaw_aa * pitch_aa).normalized();
    queue_.push({ q, timestamp });
}

} // namespace io