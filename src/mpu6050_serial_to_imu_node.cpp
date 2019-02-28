#include <geometry_msgs/Quaternion.h>
#include <ros/ros.h>
#include <serial/serial.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Temperature.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <string>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <iostream>

bool zero_orientation_set = false;

bool set_zero_orientation(std_srvs::Empty::Request &,
                          std_srvs::Empty::Response &)
{
    ROS_INFO("Zero Orientation Set.");
    zero_orientation_set = false;
    return true;
}

int main(int argc, char **argv)
{
    serial::Serial ser;
    std::string port;
    std::string tf_parent_frame_id;
    std::string tf_frame_id;
    std::string frame_id;
    double time_offset_in_seconds;
    bool broadcast_tf;
    double linear_acceleration_stddev;
    double angular_velocity_stddev;
    double orientation_stddev;
    uint8_t last_received_message_number;
    bool received_message = false;
    int data_packet_start;

    tf::Quaternion orientation;
    tf::Quaternion zero_orientation;

    ros::init(argc, argv, "mpu6050_serial_to_imu_node");

    ros::NodeHandle private_node_handle("~");
    private_node_handle.param<std::string>("port", port, "/dev/ttyUSB0");
    private_node_handle.param<std::string>("tf_parent_frame_id", tf_parent_frame_id, "imu_base");
    private_node_handle.param<std::string>("tf_frame_id", tf_frame_id, "imu_link");
    private_node_handle.param<std::string>("frame_id", frame_id, "imu_link");
    private_node_handle.param<double>("time_offset_in_seconds", time_offset_in_seconds, 0.0);
    private_node_handle.param<bool>("broadcast_tf", broadcast_tf, true);
    private_node_handle.param<double>("linear_acceleration_stddev", linear_acceleration_stddev, 0.0);
    private_node_handle.param<double>("angular_velocity_stddev", angular_velocity_stddev, 0.0);
    private_node_handle.param<double>("orientation_stddev", orientation_stddev, 0.0);

    ros::NodeHandle nh("imu");
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("data", 100);
    ros::Publisher imu_temperature_pub = nh.advertise<sensor_msgs::Temperature>("temperature", 50);
    ros::ServiceServer service = nh.advertiseService("set_zero_orientation", set_zero_orientation);

    ros::Rate r(200); // 200 hz

    sensor_msgs::Imu imu;

    imu.linear_acceleration_covariance[0] = linear_acceleration_stddev;
    imu.linear_acceleration_covariance[4] = linear_acceleration_stddev;
    imu.linear_acceleration_covariance[8] = linear_acceleration_stddev;

    imu.angular_velocity_covariance[0] = angular_velocity_stddev;
    imu.angular_velocity_covariance[4] = angular_velocity_stddev;
    imu.angular_velocity_covariance[8] = angular_velocity_stddev;

    imu.orientation_covariance[0] = orientation_stddev;
    imu.orientation_covariance[4] = orientation_stddev;
    imu.orientation_covariance[8] = orientation_stddev;

    sensor_msgs::Temperature temperature_msg;
    temperature_msg.variance = 0;

    static tf::TransformBroadcaster tf_br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(0, 0, 0));

    std::string input;
    std::string read;


    std::uint8_t header[1], body[10];
    std::uint8_t counter = 0, read_size = 0;
    bool set_angular_velocity = false;
    bool set_linear_acceleration = false;
    bool set_orientation = false;
    double angular[3], acceleration[3], angle[3];
    geometry_msgs::Quaternion q;

    while (ros::ok())
    {
        try
        {
            if (ser.isOpen())
            {
                if (ser.available())
                {
                    // read 2 byte data to find header
                    read_size = ser.read(header, 1);
                    if (read_size != 1)
                    {
                        continue;
                    }

                    // header 0x55
                    if (header[0] != 0x55)
                    {
                        continue;
                    }

                    // find the header, read body
                    read_size = ser.read(body, 10);
                    if (read_size != 10)
                    {
                        continue;
                    }

                    switch (body[0])
                    {
                    case 0x51:
                        // acceleration
                        acceleration[0] = ((short)body[1] << 8 | body[2]) / 32768 * 16 * 9.8;
                        acceleration[1] = ((short)body[3] << 8 | body[4]) / 32768 * 16 * 9.8;
                        acceleration[2] = ((short)body[5] << 8 | body[6]) / 32768 * 16 * 9.8;
                        set_linear_acceleration = true;
                        break;
                    case 0x52:
                        // angular
                        angular[0] = ((short)body[1] << 8 | body[2]) / 32768 * 2000 / 57.296;
                        angular[1] = ((short)body[3] << 8 | body[4]) / 32768 * 2000 / 57.296;
                        angular[2] = ((short)body[5] << 8 | body[6]) / 32768 * 2000 / 57.296;
                        set_angular_velocity = true;
                        break;
                    case 0x53:
                        // angle
                        angle[0] = ((short)body[1] << 8 | body[2]) / 32768 * 180 / 57.296;
                        angle[1] = ((short)body[3] << 8 | body[4]) / 32768 * 180 / 57.296;
                        angle[2] = ((short)body[5] << 8 | body[6]) / 32768 * 180 / 57.296;
                        q = tf::createQuaternionMsgFromRollPitchYaw(angle[0], angle[1], angle[2]);
                        set_orientation = true;
                        break;

                    default:
                        continue;
                        break;
                    }

                    if (set_linear_acceleration && set_angular_velocity && set_orientation)
                    {
                        imu.angular_velocity.x = angular[0];
                        imu.angular_velocity.y = angular[1];
                        imu.angular_velocity.z = angular[2];

                        imu.linear_acceleration.x = acceleration[0];
                        imu.linear_acceleration.y = acceleration[1];
                        imu.linear_acceleration.z = acceleration[2];

                        imu.orientation.x = q.x;
                        imu.orientation.y = q.y;
                        imu.orientation.z = q.z;
                        imu.orientation.w = q.w;

                        ros::Time measurement_time = ros::Time::now() + ros::Duration(time_offset_in_seconds);
                        imu.header.frame_id = frame_id;
                        imu.header.stamp = measurement_time;

                        set_linear_acceleration = false;
                        set_angular_velocity = false;
                        set_orientation = false;
                        imu_pub.publish(imu);
                    }
                }
            }
            else
            {
                // try and open the serial port
                try
                {
                    ser.setPort(port);
                    ser.setBaudrate(9600);
                    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
                    ser.setTimeout(to);
                    ser.open();
                }
                catch (serial::IOException &e)
                {
                    ROS_ERROR_STREAM("Unable to open serial port " << ser.getPort() << ". Trying again in 5 seconds.");
                    ros::Duration(5).sleep();
                }

                if (ser.isOpen())
                {
                    ROS_DEBUG_STREAM("Serial port " << ser.getPort() << " initialized and opened.");
                }
            }
        }
        catch (serial::IOException &e)
        {
            ROS_ERROR_STREAM("Error reading from the serial port " << ser.getPort() << ". Closing connection.");
            ser.close();
        }
        ros::spinOnce();
        r.sleep();
    }
}
