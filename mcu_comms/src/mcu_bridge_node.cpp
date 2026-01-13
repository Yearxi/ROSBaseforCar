#include <ros/ros.h>
#include <serial/serial.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h> // 必须包含此头文件
#include <tf/transform_broadcaster.h>

class MCUBridge {
public:
    MCUBridge() : x_(0.0), y_(0.0), th_(0.0), last_goal_cmd_(0x00) {
        ros::NodeHandle nh;
        ros::NodeHandle private_nh("~");

        std::string port;
        private_nh.param<std::string>("port", port, "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B0B027572-if00");
        
        // 读取 Goal 1 参数，如果找不到则使用默认值
        private_nh.param<double>("goals/goal1/x", g1.x, 0.0);
        private_nh.param<double>("goals/goal1/y", g1.y, 0.0);
        private_nh.param<double>("goals/goal1/yaw", g1.yaw, 0.0);

        // 读取 Goal 2 参数
        private_nh.param<double>("goals/goal2/x", g2.x, 0.0);
        private_nh.param<double>("goals/goal2/y", g2.y, 0.0);
        private_nh.param<double>("goals/goal2/yaw", g2.yaw, 0.0);

        try {
            ser.setPort(port);
            ser.setBaudrate(115200);
            serial::Timeout to = serial::Timeout::simpleTimeout(1000);
            ser.setTimeout(to);
            ser.open();
        } catch (serial::IOException& e) {
            ROS_ERROR_STREAM("Unable to open port: " << port);
        }

        cmd_sub = nh.subscribe("cmd_vel", 10, &MCUBridge::cmdCallback, this);
        imu_pub = nh.advertise<sensor_msgs::Imu>("imu/data_raw", 10);
        odom_pub = nh.advertise<nav_msgs::Odometry>("wheel_odom", 10);
        // 发布给 move_base 的简单目标话题
        goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
        
        last_time_ = ros::Time::now();
    }

    void cmdCallback(const geometry_msgs::Twist::ConstPtr& msg) {
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
        ser.write(send_buf, 11);
    }

    void spin() {
        ros::Rate loop_rate(50);
        while (ros::ok()) {
            if (ser.available() >= 24) {
                uint8_t header;
                ser.read(&header, 1);
                if (header == 0x7B) {
                    std::vector<uint8_t> data;
                    ser.read(data, 23);
                    if (data[22] == 0x7D) {
                        process_mcu_data(data); // 激活处理函数
                    }
                }
            }
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

private:
    struct Goal { double x, y, yaw; };
    Goal g1, g2;
    serial::Serial ser;
    ros::Subscriber cmd_sub;
    ros::Publisher imu_pub, odom_pub, goal_pub;

    double x_, y_, th_;
    ros::Time last_time_;
    uint8_t last_goal_cmd_; // 用于边缘触发检测

    void sendNavGoal(uint8_t goal_id) {
        // 只有当信号发生变化时才发布（从 0 变到 ID）
        if (goal_id == last_goal_cmd_) return;

        geometry_msgs::PoseStamped goal;
        goal.header.stamp = ros::Time::now();
        goal.header.frame_id = "map"; 

        bool valid = false;
        if (goal_id == 0x01) {
            ROS_INFO("MCU Triggered: Going to Goal 1");
            goal.pose.position.x = 1.5; 
            goal.pose.position.y = 0.5;
            goal.pose.orientation.w = 1.0;
            valid = true;
        } 
        else if (goal_id == 0x02) {
            ROS_INFO("MCU Triggered: Going to Goal 2");
            goal.pose.position.x = -1.0;
            goal.pose.position.y = 2.0;
            goal.pose.orientation.w = 1.0;
            valid = true;
        }

        if (valid) goal_pub.publish(goal);
        last_goal_cmd_ = goal_id; // 更新状态
    }

    void process_mcu_data(const std::vector<uint8_t>& data) {
        ros::Time current_time = ros::Time::now();
        double dt = (current_time - last_time_).toSec();
        if (dt <= 0) dt = 0.02;

        int16_t vx_raw = (data[1] << 8) | data[2];
        int16_t vy_raw = (data[3] << 8) | data[4];
        int16_t vz_raw = (data[5] << 8) | data[6];

        double vx = vx_raw / 1000.0;
        double vy = vy_raw / 1000.0;
        double vth = vz_raw / 1000.0;

        double delta_x = (vx * cos(th_) - vy * sin(th_)) * dt;
        double delta_y = (vx * sin(th_) + vy * cos(th_)) * dt;
        double delta_th = vth * dt;

        x_ += delta_x; y_ += delta_y; th_ += delta_th;

        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";
        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(th_);
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.angular.z = vth;
        for(int i=0; i<36; i+=7) odom.pose.covariance[i] = 0.001;
        odom_pub.publish(odom);

        sensor_msgs::Imu imu_msg;
        imu_msg.header.stamp = current_time;
        imu_msg.header.frame_id = "imu_link";
        int16_t ax = (data[7] << 8) | data[8];
        int16_t ay = (data[9] << 8) | data[10];
        int16_t az = (data[11] << 8) | data[12];
        imu_msg.linear_acceleration.x = (ax / 16384.0) * 9.81;
        imu_msg.linear_acceleration.y = (ay / 16384.0) * 9.81;
        imu_msg.linear_acceleration.z = (az / 16384.0) * 9.81;
        imu_msg.orientation.w = 1.0;
        imu_pub.publish(imu_msg);

        // 处理导航指令
        uint8_t goal_cmd = data[19]; 
        sendNavGoal(goal_cmd);
        if (goal_cmd == 0x00) last_goal_cmd_ = 0x00; // 按钮松开重置状态
        
        last_time_ = current_time;
    }
}; // 类定义结束

int main(int argc, char** argv) {
    ros::init(argc, argv, "mcu_bridge");
    MCUBridge bridge;
    bridge.spin();
    return 0;
}