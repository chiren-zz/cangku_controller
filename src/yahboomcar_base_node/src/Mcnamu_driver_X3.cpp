/*
 * Mcnamu_driver_X3.cpp
 * C++ ROS2 硬件驱动节点 - 替代 Python 版 Mcnamu_driver_X3.py
 *
 * 功能：
 *   1. 通过串口与 STM32 下位机通信（/dev/myserial, 115200）
 *   2. 订阅 /cmd_vel → 发送运动指令给 STM32
 *   3. 接收 STM32 上报的速度/IMU/编码器数据
 *   4. 发布 /vel_raw, /imu/data_raw, /imu/mag, /voltage, /joint_states
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/bool.hpp>

// Linux 串口头文件
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

// ══════════════════════════════════════════════════════
//  STM32 串口协议常量（来自 Rosmaster_Lib.py）
// ══════════════════════════════════════════════════════

// 帧头
static constexpr uint8_t HEAD       = 0xFF;
static constexpr uint8_t DEVICE_ID  = 0xFC;
static constexpr uint8_t COMPLEMENT = 257 - DEVICE_ID;  // = 1，用于校验和计算

// 发送给 STM32 的功能字
static constexpr uint8_t FUNC_BEEP         = 0x02;  // 蜂鸣器
static constexpr uint8_t FUNC_RGB_EFFECT   = 0x06;  // RGB灯效
static constexpr uint8_t FUNC_MOTION       = 0x12;  // 运动控制（vx, vy, vz）
static constexpr uint8_t FUNC_SET_CAR_TYPE = 0x15;  // 设置小车型号

// STM32 上报的功能字（STM32 → Jetson）
static constexpr uint8_t FUNC_REPORT_SPEED   = 0x0A;  // vx, vy, vz, 电池电压
static constexpr uint8_t FUNC_REPORT_MPU_RAW = 0x0B;  // 陀螺仪 + 加速度计 + 磁力计
static constexpr uint8_t FUNC_REPORT_IMU_ATT = 0x0C;  // roll, pitch, yaw（姿态角）
static constexpr uint8_t FUNC_REPORT_ENCODER = 0x0D;  // 四路电机编码器计数

static constexpr uint8_t CAR_TYPE_X3 = 0x01;  // X3 型号代码

// ══════════════════════════════════════════════════════
//  驱动节点类
// ══════════════════════════════════════════════════════

class YahboomcarDriver : public rclcpp::Node
{
public:
    YahboomcarDriver() : Node("driver_node")
    {
        // ── 1. 声明并读取参数 ──────────────────────────
        declare_parameter("serial_port",   "/dev/myserial");
        declare_parameter("imu_link",      "imu_link");
        declare_parameter("Prefix",        "");
        declare_parameter("xlinear_limit", 1.0);
        declare_parameter("ylinear_limit", 1.0);
        declare_parameter("angular_limit", 5.0);

        serial_port_    = get_parameter("serial_port").as_string();
        imu_link_       = get_parameter("imu_link").as_string();
        prefix_         = get_parameter("Prefix").as_string();
        xlinear_limit_  = get_parameter("xlinear_limit").as_double();
        ylinear_limit_  = get_parameter("ylinear_limit").as_double();
        angular_limit_  = get_parameter("angular_limit").as_double();

        RCLCPP_INFO(get_logger(), "serial_port: %s", serial_port_.c_str());

        // ── 2. 打开串口 ───────────────────────────────
        if (!openSerial(serial_port_)) {
            RCLCPP_ERROR(get_logger(), "串口打开失败: %s", serial_port_.c_str());
            return;
        }
        RCLCPP_INFO(get_logger(), "串口已打开: %s  115200bps", serial_port_.c_str());

        // 告诉 STM32：当前小车型号是 X3
        setCarType(CAR_TYPE_X3);

        // ── 3. 创建订阅者 ─────────────────────────────
        // 订阅导航/遥控发来的速度指令
        sub_cmd_vel_ = create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 1,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                cmdVelCallback(msg);
            });

        // 订阅 RGB 灯控制
        sub_rgb_ = create_subscription<std_msgs::msg::Int32>(
            "RGBLight", 100,
            [this](const std_msgs::msg::Int32::SharedPtr msg) {
                for (int i = 0; i < 3; i++) setRGBEffect(msg->data, 6, 1);
            });

        // 订阅蜂鸣器控制
        sub_buzzer_ = create_subscription<std_msgs::msg::Bool>(
            "Buzzer", 100,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                setBeep(msg->data ? 1 : 0);
            });

        // ── 4. 创建发布者 ─────────────────────────────
        // 发布编码器速度（给 base_node_X3 计算里程计）
        pub_vel_raw_ = create_publisher<geometry_msgs::msg::Twist>("vel_raw", 50);
        // 发布 IMU 原始数据
        pub_imu_     = create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 100);
        // 发布磁力计数据
        pub_mag_     = create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", 100);
        // 发布电池电压
        pub_voltage_ = create_publisher<std_msgs::msg::Float32>("voltage", 100);
        // 发布关节状态（给 robot_state_publisher 用）
        pub_joints_  = create_publisher<sensor_msgs::msg::JointState>("joint_states", 100);

        // ── 5. 启动串口接收后台线程 ───────────────────
        running_ = true;
        receive_thread_ = std::thread(&YahboomcarDriver::receiveThread, this);

        // ── 6. 创建定时器，10Hz 发布传感器数据 ────────
        timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            [this]() { pubDataCallback(); });

        RCLCPP_INFO(get_logger(), "Yahboom X3 驱动节点启动完成！");
    }

    ~YahboomcarDriver()
    {
        running_ = false;
        if (receive_thread_.joinable())
            receive_thread_.join();
        if (fd_ >= 0)
            close(fd_);
    }

private:
    // ══════════════════════════════════════════════════
    //  串口操作
    // ══════════════════════════════════════════════════

    int fd_ = -1;
    std::string serial_port_;
    std::mutex serial_write_mutex_;  // 写串口时加锁，防止多线程冲突

    bool openSerial(const std::string &port)
    {
        // O_RDWR: 读写模式  O_NOCTTY: 不作为控制终端  O_SYNC: 同步写入
        fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (fd_ < 0) return false;

        struct termios tty;
        memset(&tty, 0, sizeof(tty));
        tcgetattr(fd_, &tty);

        // 波特率 115200
        cfsetospeed(&tty, B115200);
        cfsetispeed(&tty, B115200);

        // 8N1 格式：8位数据位，无校验位，1位停止位
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_cflag &= ~PARENB;    // 无校验
        tty.c_cflag &= ~CSTOPB;   // 1位停止位
        tty.c_cflag |= CREAD | CLOCAL;

        // 原始模式（不处理特殊字符）
        tty.c_lflag = 0;
        tty.c_iflag = 0;
        tty.c_oflag = 0;

        // 阻塞读取，至少读到1字节才返回
        tty.c_cc[VMIN]  = 1;
        tty.c_cc[VTIME] = 0;

        tcflush(fd_, TCIFLUSH);
        tcsetattr(fd_, TCSANOW, &tty);
        return true;
    }

    // 阻塞读取1字节
    uint8_t readByte()
    {
        uint8_t b = 0;
        read(fd_, &b, 1);
        return b;
    }

    // 发送命令帧给 STM32
    // 调用前 cmd 已填好 [HEAD, DEVICE_ID, 0x00, func, data...]
    // 本函数自动填写长度字段和校验和
    void writeCMD(std::vector<uint8_t> &cmd)
    {
        // cmd[2] = 帧总长度 - 1（不含最后的校验字节）
        cmd[2] = static_cast<uint8_t>(cmd.size() - 1);

        // 校验和 = (COMPLEMENT + 所有字节之和) & 0xFF
        // COMPLEMENT = 257 - 0xFC = 1
        uint32_t cs = COMPLEMENT;
        for (auto b : cmd) cs += b;
        cmd.push_back(static_cast<uint8_t>(cs & 0xFF));

        std::lock_guard<std::mutex> lock(serial_write_mutex_);
        write(fd_, cmd.data(), cmd.size());
        usleep(2000);  // 等待 2ms，避免 STM32 丢包
    }

    // ══════════════════════════════════════════════════
    //  传感器数据（由接收线程写入，由定时器读取）
    // ══════════════════════════════════════════════════

    std::mutex data_mutex_;

    // 速度数据（FUNC_REPORT_SPEED）
    double vx_ = 0.0, vy_ = 0.0, vz_ = 0.0;
    double battery_ = 0.0;

    // IMU 数据（FUNC_REPORT_MPU_RAW）
    double ax_ = 0.0, ay_ = 0.0, az_ = 0.0;  // 加速度 m/s²
    double gx_ = 0.0, gy_ = 0.0, gz_ = 0.0;  // 角速度 rad/s
    double mx_ = 0.0, my_ = 0.0, mz_ = 0.0;  // 磁力计

    // ══════════════════════════════════════════════════
    //  串口接收线程（后台持续运行）
    // ══════════════════════════════════════════════════

    std::thread receive_thread_;
    std::atomic<bool> running_{false};

    void receiveThread()
    {
        while (running_) {
            // 等待帧头 0xFF
            uint8_t h1 = readByte();
            if (h1 != HEAD) continue;

            // 验证第二字节 0xFB（= DEVICE_ID - 1）
            uint8_t h2 = readByte();
            if (h2 != static_cast<uint8_t>(DEVICE_ID - 1)) continue;

            // 读取长度字段和功能字
            uint8_t ext_len  = readByte();  // 功能字 + 数据 + 校验 的总长度
            uint8_t ext_type = readByte();  // 功能字，决定数据含义

            int data_len = ext_len - 2;     // 纯数据字节数（不含功能字和校验）
            if (data_len < 0) continue;

            // 读取数据字节
            std::vector<uint8_t> data(data_len);
            for (auto &b : data) b = readByte();

            // 验证校验和
            // 校验规则：(ext_len + ext_type + data[0..n-2]) % 256 == data[n-1]
            uint32_t check = ext_len + ext_type;
            for (int i = 0; i < data_len - 1; i++) check += data[i];
            if ((check & 0xFF) != data[data_len - 1]) continue;

            // 解析数据
            parseData(ext_type, data);
        }
    }

    // 根据功能字解析数据帧，存入成员变量
    void parseData(uint8_t type, const std::vector<uint8_t> &d)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);

        if (type == FUNC_REPORT_SPEED) {
            // 格式：vx(int16) + vy(int16) + vz(int16) + battery(uint8)
            // int16 小端字节序，除以 1000 得到 m/s
            vx_ = static_cast<int16_t>(d[0] | (d[1] << 8)) / 1000.0;
            vy_ = static_cast<int16_t>(d[2] | (d[3] << 8)) / 1000.0;
            vz_ = static_cast<int16_t>(d[4] | (d[5] << 8)) / 1000.0;
            battery_ = d[6] / 10.0;  // 单位：V

        } else if (type == FUNC_REPORT_MPU_RAW) {
            // 陀螺仪：±500dps → ÷3754.9 = rad/s
            // 加速度：±2g    → ÷1671.84 = m/s²
            static constexpr double gyro_ratio  = 1.0 / 3754.9;
            static constexpr double accel_ratio = 1.0 / 1671.84;

            gx_ = static_cast<int16_t>(d[0]  | (d[1]  << 8)) *  gyro_ratio;
            gy_ = static_cast<int16_t>(d[2]  | (d[3]  << 8)) * -gyro_ratio;
            gz_ = static_cast<int16_t>(d[4]  | (d[5]  << 8)) * -gyro_ratio;
            ax_ = static_cast<int16_t>(d[6]  | (d[7]  << 8)) * accel_ratio;
            ay_ = static_cast<int16_t>(d[8]  | (d[9]  << 8)) * accel_ratio;
            az_ = static_cast<int16_t>(d[10] | (d[11] << 8)) * accel_ratio;
            mx_ = static_cast<int16_t>(d[12] | (d[13] << 8));
            my_ = static_cast<int16_t>(d[14] | (d[15] << 8));
            mz_ = static_cast<int16_t>(d[16] | (d[17] << 8));
        }
        // FUNC_REPORT_IMU_ATT 和 FUNC_REPORT_ENCODER 暂未使用，可按需扩展
    }

    // ══════════════════════════════════════════════════
    //  发送指令给 STM32
    // ══════════════════════════════════════════════════

    void setCarType(uint8_t car_type)
    {
        // 帧：[FF][FC][len][15][car_type][5F][checksum]
        // 0x5F = 永久保存标志
        std::vector<uint8_t> cmd = {HEAD, DEVICE_ID, 0x00, FUNC_SET_CAR_TYPE, car_type, 0x5F};
        writeCMD(cmd);
    }

    void setCarMotion(double vx, double vy, double vz)
    {
        // 速度 × 1000 打包成 int16 小端字节序
        auto pack16 = [](double v, std::vector<uint8_t> &buf) {
            int16_t val = static_cast<int16_t>(v * 1000.0);
            buf.push_back(static_cast<uint8_t>(val & 0xFF));
            buf.push_back(static_cast<uint8_t>((val >> 8) & 0xFF));
        };

        std::vector<uint8_t> cmd = {HEAD, DEVICE_ID, 0x00, FUNC_MOTION, CAR_TYPE_X3};
        pack16(vx, cmd);
        pack16(vy, cmd);
        pack16(vz, cmd);
        writeCMD(cmd);
    }

    void setBeep(int on_time)
    {
        int16_t v = static_cast<int16_t>(on_time);
        std::vector<uint8_t> cmd = {
            HEAD, DEVICE_ID, 0x00, FUNC_BEEP,
            static_cast<uint8_t>(v & 0xFF),
            static_cast<uint8_t>((v >> 8) & 0xFF)
        };
        writeCMD(cmd);
    }

    void setRGBEffect(int effect, int speed, int parm)
    {
        std::vector<uint8_t> cmd = {
            HEAD, DEVICE_ID, 0x00, FUNC_RGB_EFFECT,
            static_cast<uint8_t>(effect & 0xFF),
            static_cast<uint8_t>(speed  & 0xFF),
            static_cast<uint8_t>(parm   & 0xFF)
        };
        writeCMD(cmd);
    }

    // ══════════════════════════════════════════════════
    //  ROS2 回调函数
    // ══════════════════════════════════════════════════

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // 限速保护
        double vx = std::clamp(msg->linear.x,   -xlinear_limit_,  xlinear_limit_);
        double vy = std::clamp(msg->linear.y,   -ylinear_limit_,  ylinear_limit_);
        double vz = std::clamp(msg->angular.z,  -angular_limit_,  angular_limit_);
        setCarMotion(vx, vy, vz);
    }

    void pubDataCallback()
    {
        auto stamp = now();

        // 加锁读取传感器数据（避免和接收线程竞争）
        double vx, vy, vz, ax, ay, az, gx, gy, gz, mx, my, mz, battery;
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            vx = vx_; vy = vy_; vz = vz_;
            ax = ax_; ay = ay_; az = az_;
            gx = gx_; gy = gy_; gz = gz_;
            mx = mx_; my = my_; mz = mz_;
            battery = battery_;
        }

        // ── 发布 /vel_raw（给 base_node_X3 计算里程计）
        geometry_msgs::msg::Twist twist;
        twist.linear.x  = vx;
        twist.linear.y  = vy;
        twist.angular.z = vz;
        pub_vel_raw_->publish(twist);

        // ── 发布 /imu/data_raw
        sensor_msgs::msg::Imu imu;
        imu.header.stamp    = stamp;
        imu.header.frame_id = imu_link_;
        imu.linear_acceleration.x = ax;
        imu.linear_acceleration.y = ay;
        imu.linear_acceleration.z = az;
        imu.angular_velocity.x = gx;
        imu.angular_velocity.y = gy;
        imu.angular_velocity.z = gz;
        // 协方差设为 -1 表示未知
        imu.orientation_covariance[0] = -1;
        pub_imu_->publish(imu);

        // ── 发布 /imu/mag
        sensor_msgs::msg::MagneticField mag;
        mag.header.stamp    = stamp;
        mag.header.frame_id = imu_link_;
        mag.magnetic_field.x = mx;
        mag.magnetic_field.y = my;
        mag.magnetic_field.z = mz;
        pub_mag_->publish(mag);

        // ── 发布 /voltage
        std_msgs::msg::Float32 vol;
        vol.data = static_cast<float>(battery);
        pub_voltage_->publish(vol);

        // ── 发布 /joint_states（URDF 需要）
        sensor_msgs::msg::JointState state;
        state.header.stamp    = stamp;
        state.header.frame_id = "joint_states";
        if (prefix_.empty()) {
            state.name = {
                "back_right_joint", "back_left_joint",
                "front_left_steer_joint",  "front_left_wheel_joint",
                "front_right_steer_joint", "front_right_wheel_joint"
            };
        } else {
            state.name = {
                prefix_ + "back_right_joint",         prefix_ + "back_left_joint",
                prefix_ + "front_left_steer_joint",   prefix_ + "front_left_wheel_joint",
                prefix_ + "front_right_steer_joint",  prefix_ + "front_right_wheel_joint"
            };
        }
        state.position.assign(6, 0.0);
        state.velocity.assign(6, 0.0);
        pub_joints_->publish(state);
    }

    // ══════════════════════════════════════════════════
    //  成员变量
    // ══════════════════════════════════════════════════

    std::string imu_link_, prefix_;
    double xlinear_limit_, ylinear_limit_, angular_limit_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr       sub_rgb_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr        sub_buzzer_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr       pub_vel_raw_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr           pub_imu_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr pub_mag_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr          pub_voltage_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr    pub_joints_;

    rclcpp::TimerBase::SharedPtr timer_;
};

// ══════════════════════════════════════════════════════
//  main
// ══════════════════════════════════════════════════════

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<YahboomcarDriver>());
    rclcpp::shutdown();
    return 0;
}
