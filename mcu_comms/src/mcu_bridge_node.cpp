#include <ros/ros.h>
#include <serial/serial.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h> // 用于发布坐标变换

class MCUBridge {
public:
    MCUBridge() : x_(0.0), y_(0.0), th_(0.0) { // 初始化坐标
        ros::NodeHandle nh;
        ros::NodeHandle private_nh("~");

        std::string port;
        private_nh.param<std::string>("port", port, "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B0B027572-if00"); // 根据你的发现改为 ACM0
        
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
        odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);
        
        last_time_ = ros::Time::now();
    }

    void cmdCallback(const geometry_msgs::Twist::ConstPtr& msg) {
        // ... (保持之前的发送逻辑不变) ...
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
                        process_mcu_data(data);
                    }
                }
            }
            // pub_zero();
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

private:
    serial::Serial ser;
    ros::Subscriber cmd_sub;
    ros::Publisher imu_pub, odom_pub;
    
    // 里程计变量
    double x_, y_, th_;
    ros::Time last_time_;

    void process_mcu_data(const std::vector<uint8_t>& data) {
        ros::Time current_time = ros::Time::now();
        double dt = (current_time - last_time_).toSec();

        // 1. 速度解析 (mm/s -> m/s)
        int16_t vx_raw = (data[1] << 8) | data[2];
        int16_t vy_raw = (data[3] << 8) | data[4];
        int16_t vz_raw = (data[5] << 8) | data[6]; // 这里通常是 angular.z

        double vx = vx_raw / 1000.0;
        double vy = vy_raw / 1000.0;
        double vth = vz_raw / 1000.0; // 假设单片机传回的是角速度

        // 2. 航位推算 (Odometry integration)
        double delta_x = (vx * cos(th_) - vy * sin(th_)) * dt;
        double delta_y = (vx * sin(th_) + vy * cos(th_)) * dt;
        double delta_th = vth * dt;

        ROS_INFO("vx:%.2f,vy:%.2f,vz:%.2f\n",vx,vy,vth);


        x_ += delta_x;
        y_ += delta_y;
        th_ += delta_th;

        // 3. 发布 Odometry 消息
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";

        // 设置位置
        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(th_);

        // 设置速度
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = vth;
        
        // 协方差填充 (robot_localization 必须要)
        for(int i=0; i<36; i++) {
            odom.pose.covariance[i] = (i%7 == 0) ? 0.1 : 0.0;
            odom.twist.covariance[i] = (i%7 == 0) ? 0.1 : 0.0;
        }

        odom_pub.publish(odom);

        // 4. 处理并发布 IMU (保持不变)
        sensor_msgs::Imu imu_msg;
        imu_msg.header.stamp = current_time;
        imu_msg.header.frame_id = "imu_link";
        // ... (ax, ay, az, gx, gy, gz 的赋值) ...
        
        // 这里需要注意：单片机 data[7]-data[12] 是加速度，data[13]-data[18] 是角速度
        int16_t ax = (data[7] << 8) | data[8];
        int16_t ay = (data[9] << 8) | data[10];
        int16_t az = (data[11] << 8) | data[12];
        imu_msg.linear_acceleration.x = (ax / 16384.0) * 9.81;
        imu_msg.linear_acceleration.y = (ay / 16384.0) * 9.81;
        imu_msg.linear_acceleration.z = (az / 16384.0) * 9.81;

        ROS_INFO("ax:%.2f,ay:%.2f,az:%.2f\n",ax,ay,az);


        imu_pub.publish(imu_msg);
        
        last_time_ = current_time;
    }

    void pub_zero(void){
        nav_msgs::Odometry odom;
        sensor_msgs::Imu imu_msg;
        odom_pub.publish(odom);
        imu_pub.publish(imu_msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "mcu_bridge");
    MCUBridge bridge;
    bridge.spin();
    return 0;
}