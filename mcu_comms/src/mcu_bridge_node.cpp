#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h> // 确保已安装 serial 库
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class MCUBridge : public rclcpp::Node {
public:
    MCUBridge() : Node("mcu_bridge"), x_(0.0), y_(0.0), th_(0.0), last_goal_cmd_(0x00) {
        // 参数读取
        this->declare_parameter<std::string>("port", "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B0B027572-if00");
        std::string port = this->get_parameter("port").as_string();

        this->declare_parameter<double>("goals.goal1.x", 1.5);
        this->declare_parameter<double>("goals.goal1.y", 0.5);
        this->declare_parameter<double>("goals.goal2.x", -1.0);
        this->declare_parameter<double>("goals.goal2.y", 2.0);

        try {
            ser_.setPort(port);
            ser_.setBaudrate(115200);
            serial::Timeout to = serial::Timeout::simpleTimeout(1000);
            ser_.setTimeout(to);
            ser_.open();
        } catch (serial::IOException& e) {
            RCLCPP_ERROR(this->get_logger(), "Unable to open port: %s", port.c_str());
        }

        // ROS 2 订阅与发布
        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&MCUBridge::cmd_callback, this, std::placeholders::_1));
        
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("wheel_odom", 10);
        goal_pub_ = this->create_publisher<geometry_msgs::msg::Pose_stamped>("/move_base_simple/goal", 10);
        
        last_time_ = this->now();
        
        // 使用定时器代替原来的 spin 循环
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20), std::bind(&MCUBridge::timer_callback, this));
    }

private:
    serial::Serial ser_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    double x_, y_, th_;
    rclcpp::Time last_time_;
    uint8_t last_goal_cmd_;

    void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        uint8_t send_buf[11];
        send_buf[0] = 0x7B;
        int16_t vx = msg->linear.x * 1000;
        int16_t vy = msg->linear.y * 1000;
        int16_t vz = msg->angular.z * 1000;
        send_buf[2] = (vx >> 8) & 0xFF;  send_buf[3] = vx & 0xFF;
        send_buf[4] = (vy >> 8) & 0xFF;  send_buf[5] = vy & 0xFF;
        send_buf[6] = (vz >> 8) & 0xFF;  send_buf[7] = vz & 0xFF;
        uint8_t checksum = 0;
        for(int i=0; i<9; i++) checksum ^= send_buf[i];
        send_buf[9] = checksum;
        send_buf[10] = 0x7D;
        ser_.write(send_buf, 11);
    }

    void timer_callback() {
        if (ser_.available() >= 24) {
            uint8_t header;
            ser_.read(&header, 1);
            if (header == 0x7B) {
                std::vector<uint8_t> data;
                ser_.read(data, 23);
                if (data[22] == 0x7D) {
                    process_mcu_data(data);
                }
            }
        }
    }

    void process_mcu_data(const std::vector<uint8_t>& data) {
        rclcpp::Time current_time = this->now();
        double dt = (current_time - last_time_).seconds();
        if (dt <= 0) dt = 0.02;

        int16_t vx_raw = (data[1] << 8) | data[2];
        int16_t vy_raw = (data[3] << 8) | data[4];
        int16_t vz_raw = (data[5] << 8) | data[6];
        int16_t ax_raw = (data[7] << 8) | data[8];
        int16_t ay_raw = (data[9] << 8) | data[10];
        int16_t gx_raw = (data[13] << 8) | data[14];
        int16_t gy_raw = (data[15] << 8) | data[16];
        int16_t gz_raw = (data[17] << 8) | data[18];
        uint8_t goal_cmd = data[19];

        double vx = vx_raw / 1000.0;
        double vy = vy_raw / 1000.0;
        double vth = vz_raw / 1000.0;
    
        x_ += (vx * cos(th_) - vy * sin(th_)) * dt;
        y_ += (vx * sin(th_) + vy * cos(th_)) * dt;
        th_ += vth * dt;

        // IMU 转换
        const double ACCEL_SCALE = 16384.0;
        const double GYRO_SCALE = 131.0;
        double ax = (ax_raw / ACCEL_SCALE) * 9.80665;
        double ay = (ay_raw / ACCEL_SCALE) * 9.80665;
        double wx = (gx_raw / GYRO_SCALE) * (M_PI / 180.0);
        double wy = (gy_raw / GYRO_SCALE) * (M_PI / 180.0);
        double wz = (gz_raw / GYRO_SCALE) * (M_PI / 180.0);

        // 创建四元数
        tf2::Quaternion q;
        q.setRPY(0, 0, th_);

        // 发布里程计
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";
        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.orientation = tf2::toMsg(q);
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.angular.z = vth;
        odom_pub_->publish(odom);

        // 发布 IMU
        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header.stamp = current_time;
        imu_msg.header.frame_id = "imu_link";
        imu_msg.linear_acceleration.x = ax; // 注意：原代码逻辑这里除以了两次 16384，已修正
        imu_msg.linear_acceleration.y = ay;
        imu_msg.linear_acceleration.z = 9.81; // 假设 Z 轴
        imu_msg.angular_velocity.x = wx;
        imu_msg.angular_velocity.y = wy;
        imu_msg.angular_velocity.z = wz;
        imu_msg.orientation = tf2::toMsg(q);
        imu_pub_->publish(imu_msg);

        handle_nav_goal(goal_cmd);
        last_time_ = current_time;
    }

    void handle_nav_goal(uint8_t goal_id) {
        if (goal_id == last_goal_cmd_) return;
        if (goal_id == 0x00) { last_goal_cmd_ = 0x00; return; }

        geometry_msgs::msg::PoseStamped goal;
        goal.header.stamp = this->now();
        goal.header.frame_id = "map";
        
        if (goal_id == 0x01) {
            goal.pose.position.x = this->get_parameter("goals.goal1.x").as_double();
            goal.pose.position.y = this->get_parameter("goals.goal1.y").as_double();
        } else if (goal_id == 0x02) {
            goal.pose.position.x = this->get_parameter("goals.goal2.x").as_double();
            goal.pose.position.y = this->get_parameter("goals.goal2.y").as_double();
        }
        goal.pose.orientation.w = 1.0;
        
        RCLCPP_INFO(this->get_logger(), "Goal triggered: %d", goal_id);
        goal_pub_->publish(goal);
        last_goal_cmd_ = goal_id;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MCUBridge>());
    rclcpp::shutdown();
    return 0;
}