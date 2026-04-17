#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/bool.hpp>

#include <termios.h>
#include <fcntl.h>
#include <unistd.h>

#include <thread>
#include <mutex>
#include <atomic>
#include <vector>
#include <cstring>
#include <cstdint>
#include <algorithm>

// ── STM32 串口协议常量 ─────────────────────────────────
static constexpr uint8_t HEAD       = 0xFF;
static constexpr uint8_t DEVICE_ID  = 0xFC;
static constexpr uint8_t COMPLEMENT = 257 - DEVICE_ID;  // = 1

// 发送给 STM32
static constexpr uint8_t FUNC_BEEP          = 0x02;
static constexpr uint8_t FUNC_RGB_EFFECT    = 0x06;
static constexpr uint8_t FUNC_MOTION        = 0x12;
static constexpr uint8_t FUNC_SET_CAR_TYPE  = 0x15;

// STM32 主动上报
static constexpr uint8_t FUNC_REPORT_SPEED   = 0x0A;
static constexpr uint8_t FUNC_REPORT_MPU_RAW = 0x0B;
static constexpr uint8_t FUNC_REPORT_IMU_ATT = 0x0C;
static constexpr uint8_t FUNC_REPORT_ENCODER = 0x0D;

static constexpr uint8_t CAR_TYPE_X3 = 0x01;

// ── 驱动节点类声明 ────────────────────────────────────

class YahboomcarDriver : public rclcpp::Node
{
public:
    YahboomcarDriver();
    ~YahboomcarDriver();

private:
    // 串口
    bool    openSerial(const std::string &port);
    uint8_t readByte();
    void    writeCMD(std::vector<uint8_t> &cmd);

    // 接收线程
    void receiveThread();
    void parseData(uint8_t type, const std::vector<uint8_t> &d);

    // 发送指令
    void setCarType(uint8_t car_type);
    void setCarMotion(double vx, double vy, double vz);
    void setBeep(int on_time);
    void setRGBEffect(int effect, int speed, int parm);

    // ROS 回调
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void pubDataCallback();

    // ── 串口 ──
    int         fd_ = -1;
    std::string serial_port_;
    std::mutex  serial_write_mutex_;

    // ── 传感器数据（接收线程写，定时器读）──
    std::mutex data_mutex_;
    double vx_ = 0.0, vy_ = 0.0, vz_ = 0.0;
    double battery_ = 0.0;
    double ax_ = 0.0, ay_ = 0.0, az_ = 0.0;
    double gx_ = 0.0, gy_ = 0.0, gz_ = 0.0;
    double mx_ = 0.0, my_ = 0.0, mz_ = 0.0;

    // ── 接收线程 ──
    std::thread       receive_thread_;
    std::atomic<bool> running_{false};

    // ── 参数 ──
    std::string imu_link_, prefix_;
    double xlinear_limit_, ylinear_limit_, angular_limit_;

    // ── ROS 接口 ──
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr      sub_rgb_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr       sub_buzzer_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr       pub_vel_raw_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr           pub_imu_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr pub_mag_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr          pub_voltage_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr    pub_joints_;

    rclcpp::TimerBase::SharedPtr timer_;
};