#include <ros/ros.h>
#include <serial/serial.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

class MCUBridge {
public:
    MCUBridge() {
        ros::NodeHandle nh;
        ros::NodeHandle private_nh("~");

        // 串口参数配置
        std::string port;
        private_nh.param<std::string>("port", port, "/dev/ttyUSB0");
        
        try {
            ser.setPort(port);
            ser.setBaudrate(115200);
            serial::Timeout to = serial::Timeout::simpleTimeout(1000);
            ser.setTimeout(to);
            ser.open();
        } catch (serial::IOException& e) {
            ROS_ERROR_STREAM("无法打开串口端口: " << port);
        }

        // 订阅控制指令
        cmd_sub = nh.subscribe("cmd_vel", 10, &MCUBridge::cmdCallback, this);
        // 发布IMU和里程计（供定位使用）
        imu_pub = nh.advertise<sensor_msgs::Imu>("imu/data_raw", 10);
        odom_pub = nh.advertise<nav_msgs::Odometry>("raw_odom", 10);
    }

    // 处理来自 ROS 的控制命令并发往单片机
    void cmdCallback(const geometry_msgs::Twist::ConstPtr& msg) {
        uint8_t send_buf[11];
        send_buf[0] = 0x7B; // 帧头
        send_buf[1] = 0;    // 模式/预留

        // 转换速度为短整型 (对应单片机 XYZ_Target_Speed_transition)
        int16_t vx = msg->linear.x * 1000;
        int16_t vy = msg->linear.y * 1000;
        int16_t vz = msg->angular.z * 1000;

        send_buf[2] = (vx >> 8) & 0xFF;  send_buf[3] = vx & 0xFF;
        send_buf[4] = (vy >> 8) & 0xFF;  send_buf[5] = vy & 0xFF;
        send_buf[6] = (vz >> 8) & 0xFF;  send_buf[7] = vz & 0xFF;
        send_buf[8] = 0; // 预留
        
        // 计算校验和
        uint8_t checksum = 0;
        for(int i=0; i<9; i++) checksum ^= send_buf[i];
        send_buf[9] = checksum;
        send_buf[10] = 0x7D; // 帧尾

        ser.write(send_buf, 11);
    }

    void spin() {
        ros::Rate loop_rate(50); // 略高于单片机 20Hz 的频率
        while (ros::ok()) {
            if (ser.available() >= 24) {
                uint8_t header;
                ser.read(&header, 1);
                if (header == 0x7B) {
                    std::vector<uint8_t> data;
                    ser.read(data, 23); // 读取剩余23字节
                    if (data[22] == 0x7D) {
                        process_mcu_data(data);
                    }
                }
            }
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

private:
    serial::Serial ser;
    ros::Subscriber cmd_sub;
    ros::Publisher imu_pub, odom_pub;

    void process_mcu_data(const std::vector<uint8_t>& data) {
        // 1. 速度解算 (对应 Send_Data.Sensor_Str)
        int16_t vx_raw = (data[1] << 8) | data[2];
        float vx = vx_raw / 1000.0;

        // 2. IMU 数据解算 (对应 Mpu6050_Data)
        // 注意：这里需要根据你单片机 MPU6050 的量程(LSB/g)进行物理单位转换
        // 假设 Accel LSB = 16384, Gyro LSB = 65.5
        sensor_msgs::Imu imu_msg;
        imu_msg.header.stamp = ros::Time::now();
        imu_msg.header.frame_id = "imu_link";
        
        int16_t ax = (data[7] << 8) | data[8];
        int16_t gx = (data[13] << 8) | data[14];
        
        imu_msg.linear_acceleration.x = (ax / 16384.0) * 9.81;
        imu_msg.angular_velocity.z = (gx / 65.5) * (M_PI / 180.0);
        
        // 填充协方差（重要：localization节点需要此项）
        for(int i=0; i<9; i++) imu_msg.linear_acceleration_covariance[i] = 0.01;
        
        imu_pub.publish(imu_msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "mcu_bridge");
    MCUBridge bridge;
    bridge.spin();
    return 0;
}