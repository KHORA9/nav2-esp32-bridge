/**
 * hanatra_control — ROS2 Node: ESP32 UART Bridge
 *
 * Bridges Nav2 cmd_vel → ESP32 motor controller via UART protocol.
 * Protocol: 0xAA/0x55 header + CRC16-Modbus
 *
 * Publishers: /hanatra/odom, /hanatra/imu/data, /hanatra/battery,
 *            /hanatra/estop_status, /hanatra/motor_status, /hanatra/robot_state
 * Subscribers: /hanatra/cmd_vel, /hanatra/estop
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <hanatra_msgs/msg/robot_state.hpp>
#include <hanatra_msgs/msg/motor_status.hpp>
#include <hanatra_msgs/msg/e_stop_status.hpp>
#include <hanatra_msgs/msg/safety_status.hpp>
#include <hanatra_msgs/msg/motor_command.hpp>

#include <serial/serial.h>
#include <cmath>
#include <cstring>
#include <vector>

using namespace std::chrono_literals;

// ============================================================================
// CONFIGURATION
// ============================================================================

static const float WHEEL_RADIUS_M      = 0.05f;
static const float WHEEL_TRACK_M        = 0.26f;   // distance between wheels
static const float MAX_LINEAR_SPEED     = 0.5f;    // m/s
static const float MAX_ANGULAR_SPEED    = 1.5f;    // rad/s
static const float ODOM_PERIOD_S       = 1.0f / 20.0f;
static const int   UART_BAUD            = 115200;
static const float BAT_FULL_VOLTAGE     = 25.2f;
static const float BAT_EMPTY_VOLTAGE    = 20.0f;

// ============================================================================
// UART PROTOCOL
// ============================================================================

static const uint8_t SOF = 0xAA;
static const uint8_t EOF_MARKER = 0x55;

static const uint8_t CMD_MOTOR_CMD   = 0x01;
static const uint8_t CMD_MOTOR_STATUS = 0x02;
static const uint8_t CMD_IMU         = 0x03;
static const uint8_t CMD_ESTOP       = 0x04;
static const uint8_t CMD_BATTERY     = 0x05;
static const uint8_t CMD_DIAGNOSTICS = 0x06;
static const uint8_t CMD_HEARTBEAT   = 0x0A;
static const uint8_t CMD_TELEMETRY   = 0x10;

static uint16_t crc16_modbus(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int j = 0; j < 8; ++j) {
            if (crc & 0x0001) { crc = (crc >> 1) ^ 0xA001; }
            else { crc >>= 1; }
        }
    }
    return crc;
}

// Build a frame: SOF + CMD + LEN + DATA + CRC16 (LE) + EOF
static std::vector<uint8_t> build_frame(uint8_t cmd, const uint8_t* data, uint8_t len) {
    std::vector<uint8_t> frame;
    frame.reserve(6 + len);
    frame.push_back(SOF);
    frame.push_back(cmd);
    frame.push_back(len);
    for (size_t i = 0; i < len; ++i) frame.push_back(data[i]);
    uint16_t crc = crc16_modbus(frame.data() + 1, len + 2); // cmd + len + data
    frame.push_back(crc & 0xFF);
    frame.push_back((crc >> 8) & 0xFF);
    frame.push_back(EOF_MARKER);
    return frame;
}

// Differential drive: cmd_vel → wheel velocities
static void cmd_vel_to_wheels(float vx, float wz,
                               int16_t& left_mm_s, int16_t& right_mm_s) {
    // v_l = (vx - wz * track/2) / radius
    // v_r = (vx + wz * track/2) / radius
    float v_l = (vx - wz * WHEEL_TRACK_M / 2.0f) / WHEEL_RADIUS_M;  // rad/s
    float v_r = (vx + wz * WHEEL_TRACK_M / 2.0f) / WHEEL_RADIUS_M;  // rad/s
    // Convert to mm/s
    v_l *= WHEEL_RADIUS_M * 1000.0f;
    v_r *= WHEEL_RADIUS_M * 1000.0f;
    // Clamp
    left_mm_s  = static_cast<int16_t>(std::clamp(v_l, -500.0f, 500.0f));
    right_mm_s = static_cast<int16_t>(std::clamp(v_r, -500.0f, 500.0f));
}

// ============================================================================
// ESP32 BRIDGE NODE
// ============================================================================

class ESP32BridgeNode : public rclcpp::Node {
public:
    ESP32BridgeNode()
        : Node("esp32_bridge"),
          odom_left_counts_(0),
          odom_right_counts_(0),
          odom_x_(0.0), odom_y_(0.0), odom_yaw_(0.0)
    {
        // Declare parameters
        this->declare_parameter<std::string>("uart_port", "/dev/ttyUSB0");
        this->declare_parameter<int>("uart_baud", UART_BAUD);
        this->declare_parameter<std::string>("namespace", "hanatra");
        this->declare_parameter<float>("wheel_radius", WHEEL_RADIUS_M);
        this->declare_parameter<float>("wheel_track", WHEEL_TRACK_M);

        std::string port = this->get_parameter("uart_port").as_string();
        int baud = this->get_parameter("uart_baud").as_int();

        RCLCPP_INFO(this->get_logger(), "Opening UART %s at %d", port.c_str(), baud);

        try {
            serial_.setPort(port);
            serial_.setBaudrate(baud);
            serial::Timeout to = serial::Timeout::simpleTimeout(100);
            serial_.setTimeout(to);
            serial_.open();
            RCLCPP_INFO(this->get_logger(), "UART opened successfully");
        } catch (const serial::IOException& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open UART: %s — running in simulation mode", e.what());
            simulation_mode_ = true;
        }

        init_publishers();
        init_subscribers();
        init_timers();
        init_transform_broadcaster();
    }

    ~ESP32BridgeNode() {
        if (serial_.isOpen()) serial_.close();
    }

private:
    bool simulation_mode_ = false;
    serial::Serial serial_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    // Odometry state
    int32_t odom_left_counts_, odom_right_counts_;
    float odom_x_, odom_y_, odom_yaw_;
    float last_left_vel_ = 0.0f, last_right_vel_ = 0.0f;

    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
    rclcpp::Publisher<hanatra_msgs::msg::MotorStatus>::SharedPtr motor_status_pub_;
    rclcpp::Publisher<hanatra_msgs::msg::EStopStatus>::SharedPtr estop_pub_;
    rclcpp::Publisher<hanatra_msgs::msg::RobotState>::SharedPtr robot_state_pub_;

    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
    rclcpp::Subscription<hanatra_msgs::msg::MotorCommand>::SharedPtr motor_cmd_sub_;

    // Timers
    rclcpp::TimerBase::SharedPtr uart_rx_timer_;
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;
    rclcpp::TimerBase::SharedPtr odom_pub_timer_;

    // Safety state
    uint8_t safety_state_ = 0;
    uint8_t estop_reason_ = 0;
    float battery_voltage_ = 24.0f;
    float battery_percent_ = 100.0f;
    bool estop_active_ = false;

    void init_publishers() {
        odom_pub_        = this->create_publisher<nav_msgs::msg::Odometry>("~/odom", 10);
        imu_pub_         = this->create_publisher<sensor_msgs::msg::Imu>("~/imu/data", 200);
        battery_pub_     = this->create_publisher<sensor_msgs::msg::BatteryState>("~/battery", 10);
        motor_status_pub_ = this->create_publisher<hanatra_msgs::msg::MotorStatus>("~/motor_status", 20);
        estop_pub_       = this->create_publisher<hanatra_msgs::msg::EStopStatus>("~/estop_status", 10);
        robot_state_pub_ = this->create_publisher<hanatra_msgs::msg::RobotState>("~/robot_state", 1);
    }

    void init_subscribers() {
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "~/cmd_vel", 10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                this->on_cmd_vel(msg);
            });

        estop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "~/estop", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                this->on_estop(msg);
            });

        motor_cmd_sub_ = this->create_subscription<hanatra_msgs::msg::MotorCommand>(
            "~/motor_cmd", 10,
            [this](const hanatra_msgs::msg::MotorCommand::SharedPtr msg) {
                this->on_motor_cmd(msg);
            });
    }

    void init_timers() {
        // UART RX polling at 50 Hz
        uart_rx_timer_ = this->create_wall_timer(
            20ms, [this]() { this->uart_rx_loop(); });

        // Heartbeat at 10 Hz
        heartbeat_timer_ = this->create_wall_timer(
            100ms, [this]() { this->send_heartbeat(); });

        // Odometry publish at 20 Hz
        odom_pub_timer_ = this->create_wall_timer(
            50ms, [this]() { this->publish_odometry(); });
    }

    void init_transform_broadcaster() {
        // tf_broadcaster_ initialized in constructor
    }

    void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg) {
        // Clamp inputs to safety limits
        float vx = std::clamp(static_cast<float>(msg->linear.x), -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED);
        float wz = std::clamp(static_cast<float>(msg->angular.z), -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);

        int16_t left_mm_s, right_mm_s;
        cmd_vel_to_wheels(vx, wz, left_mm_s, right_mm_s);

        // Build UART frame
        uint8_t data[4];
        std::memcpy(data,     &left_mm_s,  2);
        std::memcpy(data + 2, &right_mm_s, 2);
        auto frame = build_frame(CMD_MOTOR_CMD, data, 4);

        if (!simulation_mode_ && serial_.isOpen()) {
            try {
                serial_.write(frame);
            } catch (const serial::IOException& e) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "UART write failed: %s", e.what());
            }
        }
    }

    void on_estop(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
            uint8_t data[1] = {0x01}; // reason: software estop
            auto frame = build_frame(CMD_ESTOP, data, 1);
            if (!simulation_mode_ && serial_.isOpen()) {
                serial_.write(frame);
            }
            estop_active_ = true;
            RCLCPP_WARN(this->get_logger(), "Software E-Stop triggered");
        }
    }

    void on_motor_cmd(const hanatra_msgs::msg::MotorCommand::SharedPtr msg) {
        int16_t left = static_cast<int16_t>(std::clamp(msg->left_velocity_mm_s, -500, 500));
        int16_t right = static_cast<int16_t>(std::clamp(msg->right_velocity_mm_s, -500, 500));
        uint8_t data[4];
        std::memcpy(data,     &left,  2);
        std::memcpy(data + 2, &right, 2);
        auto frame = build_frame(CMD_MOTOR_CMD, data, 4);
        if (!simulation_mode_ && serial_.isOpen()) serial_.write(frame);
    }

    void send_heartbeat() {
        static uint8_t seq = 0;
        uint8_t data[2] = {seq++, safety_state_};
        auto frame = build_frame(CMD_HEARTBEAT, data, 2);
        if (!simulation_mode_ && serial_.isOpen()) {
            try { serial_.write(frame); } catch (...) {}
        }
    }

    void uart_rx_loop() {
        if (simulation_mode_ || !serial_.isOpen()) return;

        try {
            size_t available = serial_.available();
            if (available == 0) return;

            std::vector<uint8_t> buf(available);
            size_t read = serial_.read(buf.data(), available);

            for (size_t i = 0; i < read; ++i) {
                parse_byte(buf[i]);
            }
        } catch (const serial::IOException& e) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "UART read error: %s", e.what());
        }
    }

    // State machine for parsing 0xAA 0x55 frames
    enum RxState { SEARCHING, GOT_SOF, GOT_LEN, GOT_PAYLOAD };
    RxState rx_state_ = SEARCHING;
    std::vector<uint8_t> rx_buf_;

    void parse_byte(uint8_t b) {
        switch (rx_state_) {
            case SEARCHING:
                if (b == SOF) {
                    rx_buf_.clear();
                    rx_buf_.push_back(b);
                    rx_state_ = GOT_SOF;
                }
                break;
            case GOT_SOF:
                if (b == EOF_MARKER) { rx_state_ = SEARCHING; }
                else { rx_buf_.push_back(b); rx_state_ = GOT_LEN; }
                break;
            case GOT_LEN: {
                uint8_t len = std::min<uint8_t>(b, 64);
                rx_buf_.push_back(b);
                if (len > 0) rx_state_ = GOT_PAYLOAD;
                else { process_frame(); rx_state_ = SEARCHING; }
                break;
            }
            case GOT_PAYLOAD:
                rx_buf_.push_back(b);
                if (rx_buf_.size() >= static_cast<size_t>(rx_buf_[2]) + 5) {
                    process_frame();
                    rx_state_ = SEARCHING;
                }
                break;
        }
    }

    void process_frame() {
        if (rx_buf_.size() < 6) return;  // need at least SOF + CMD + LEN + CRC16 + EOF
        uint8_t cmd = rx_buf_[1];
        uint8_t len = rx_buf_[2];
        const uint8_t* payload = rx_buf_.data() + 3;

        // Verify CRC
        uint16_t received_crc = payload[len] | (payload[len + 1] << 8);
        uint16_t computed_crc = crc16_modbus(rx_buf_.data() + 1, len + 3);
        if (computed_crc != received_crc) {
            RCLCPP_WARN(this->get_logger(), "CRC mismatch on frame 0x%02X", cmd);
            return;
        }

        // Process by command
        switch (cmd) {
            case CMD_MOTOR_STATUS: parse_motor_status(payload, len); break;
            case CMD_IMU:           parse_imu(payload, len); break;
            case CMD_BATTERY:      parse_battery(payload, len); break;
            case CMD_DIAGNOSTICS:   parse_diagnostics(payload, len); break;
            case CMD_HEARTBEAT:     parse_heartbeat(payload, len); break;
        }
    }

    void parse_motor_status(const uint8_t* payload, uint8_t len) {
        if (len < 10) return;
        int32_t left_counts  = payload[0]  | (payload[1] << 8)  | (payload[2] << 16)  | (payload[3] << 24);
        int32_t right_counts = payload[4]  | (payload[5] << 8)  | (payload[6] << 16)  | (payload[7] << 24);
        int16_t left_rpm    = payload[8]  | (payload[9] << 8);

        odom_left_counts_  = left_counts;
        odom_right_counts_ = right_counts;

        // Update odometry
        static const float COUNTS_PER_M = 1.0f / (WHEEL_RADIUS_M * 2.0f * M_PI * 400.0f);
        int32_t dl = left_counts - last_left_vel_;
        int32_t dr = right_counts - last_right_vel_;
        float dl_m = dl * COUNTS_PER_M;
        float dr_m = dr * COUNTS_PER_M;
        float dist = (dl_m + dr_m) / 2.0f;
        odom_x_ += dist * std::cos(odom_yaw_);
        odom_y_ += dist * std::sin(odom_yaw_);
        float dth = (dr_m - dl_m) / WHEEL_TRACK_M;
        odom_yaw_ += dth;
        last_left_vel_ = left_counts;
        last_right_vel_ = right_counts;

        // Publish motor status
        auto status = hanatra_msgs::msg::MotorStatus();
        status.left_encoder_counts = left_counts;
        status.right_encoder_counts = right_counts;
        status.left_rpm = left_rpm;
        status.right_rpm = payload[10] | (payload[11] << 8);
        motor_status_pub_->publish(status);
    }

    void parse_imu(const uint8_t* payload, uint8_t len) {
        if (len < 12) return;
        auto imu_msg = sensor_msgs::msg::Imu();
        imu_msg.header.stamp = this->get_clock()->now();
        imu_msg.header.frame_id = "imu_link";

        // Parse MPU6050 int16 scaled values
        int16_t ax = payload[0]  | (payload[1] << 8);
        int16_t ay = payload[2]  | (payload[3] << 8);
        int16_t az = payload[4]  | (payload[5] << 8);
        int16_t gx = payload[6]  | (payload[7] << 8);
        int16_t gy = payload[8]  | (payload[9] << 8);
        int16_t gz = payload[10] | (payload[11] << 8);

        // Scale: accelerometer 16384 LSB/g → m/s², gyroscope 131 LSB/°/s → rad/s
        imu_msg.linear_acceleration.x = ax / 16384.0f * 9.81f;
        imu_msg.linear_acceleration.y = ay / 16384.0f * 9.81f;
        imu_msg.linear_acceleration.z = az / 16384.0f * 9.81f;
        imu_msg.angular_velocity.x = gx / 131.0f * M_PI / 180.0f;
        imu_msg.angular_velocity.y = gy / 131.0f * M_PI / 180.0f;
        imu_msg.angular_velocity.z = gz / 131.0f * M_PI / 180.0f;
        imu_pub_->publish(imu_msg);
    }

    void parse_battery(const uint8_t* payload, uint8_t len) {
        if (len < 3) return;
        uint16_t voltage_mv = payload[0] | (payload[1] << 8);
        battery_voltage_ = voltage_mv / 1000.0f;
        battery_percent_ = payload[2];

        auto bat = sensor_msgs::msg::BatteryState();
        bat.header.stamp = this->get_clock()->now();
        bat.header.frame_id = "battery";
        bat.voltage = battery_voltage_;
        bat.percentage = battery_percent_ / 100.0f;
        bat.present = true;
        battery_pub_->publish(bat);
    }

    void parse_diagnostics(const uint8_t* payload, uint8_t len) {
        if (len < 5) return;
        float motor_temp_c = 0;
        std::memcpy(&motor_temp_c, payload, 4);
        uint8_t fault_flags = payload[4];

        auto rs = hanatra_msgs::msg::RobotState();
        rs.battery_voltage = battery_voltage_;
        rs.battery_percent = battery_percent_;
        rs.motor_temp_c = motor_temp_c;
        rs.encoder_fault = fault_flags & 0x01;
        rs.imu_fault = fault_flags & 0x02;
        rs.battery_fault = fault_flags & 0x04;
        rs.state = safety_state_;
        robot_state_pub_->publish(rs);
    }

    void parse_heartbeat(const uint8_t* payload, uint8_t len) {
        if (len < 2) return;
        safety_state_ = payload[1];
    }

    void publish_odometry() {
        auto odom = nav_msgs::msg::Odometry();
        odom.header.stamp = this->get_clock()->now();
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";

        odom.pose.pose.position.x = odom_x_;
        odom.pose.pose.position.y = odom_y_;
        odom.pose.pose.orientation.z = std::sin(odom_yaw_ / 2.0f);
        odom.pose.pose.orientation.w = std::cos(odom_yaw_ / 2.0f);
        odom.twist.twist.linear.x = last_left_vel_ * 0.001f; // rough estimate
        odom_pub_->publish(odom);

        // Broadcast tf: odom → base_link
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = this->get_clock()->now();
        tf.header.frame_id = "odom";
        tf.child_frame_id = "base_link";
        tf.transform.translation.x = odom_x_;
        tf.transform.translation.y = odom_y_;
        tf.transform.rotation.z = std::sin(odom_yaw_ / 2.0f);
        tf.transform.rotation.w = std::cos(odom_yaw_ / 2.0f);
        tf_broadcaster_.sendTransform(tf);
    }
};

// ============================================================================
// MAIN
// ============================================================================

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ESP32BridgeNode>());
    rclcpp::shutdown();
    return 0;
}
