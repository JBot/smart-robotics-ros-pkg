#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float32.h>

std_msgs::Float32 tspeed;

#define UNSEEN_VALUE  2600

void tspeedCallback(const std_msgs::Float32::ConstPtr & pose)
{
        tspeed.data = pose->data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "neato_xv11_lds_publisher");
    ros::NodeHandle n;
    ros::NodeHandle priv_nh("~");

    ros::Subscriber tspeed_sub_;
    tspeed_sub_ = n.subscribe < std_msgs::Float32 > ("/tspeed", 30, tspeedCallback);

    std::string port;
    int baud_rate;
    std::string frame_id;

    priv_nh.param("port", port, std::string("/dev/ttyUSB0"));
    priv_nh.param("baud_rate", baud_rate, 115200);
    priv_nh.param("frame_id", frame_id, std::string("base_laser"));

    boost::asio::io_service io;
    boost::array < uint8_t, 16 > raw_bytes_;
    boost::array < uint8_t, 2 > speed_;

    ros::Rate loop_rate(100);

    try {
        boost::asio::serial_port serial_(io, port);
        serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));

	//tf::TransformBroadcaster broadcaster;

        ros::Publisher laser_pub = n.advertise < sensor_msgs::LaserScan > ("scan", 50);
        sensor_msgs::LaserScan scan;

        scan.header.frame_id = "/base_laser";
        scan.header.stamp = ros::Time::now();
        scan.angle_min = 0.0;
        scan.angle_max = 2.0 * M_PI;
        scan.angle_increment = (2.0 * M_PI / 360.0);
        scan.time_increment = (1 / 6) / 360;
	scan.scan_time = 1 / 6;
        scan.range_min = 0.015;
        scan.range_max = 5;

        scan.set_ranges_size(360);
        scan.set_intensities_size(360);


        uint8_t init_level = 0;
	uint8_t index;
	uint8_t temp_char;

        while (ros::ok()) {

	    ros::spinOnce();
            //scan.header.frame_id = frame_id;

            if (init_level == 0) {
                boost::asio::read(serial_, boost::asio::buffer(&temp_char, 1));
                // start byte
                if (temp_char == 0xFA)
                    init_level = 1;
                else
                    init_level = 0;
            } else if (init_level == 1) {
                // position index 
                boost::asio::read(serial_, boost::asio::buffer(&temp_char, 1));

                if (temp_char >= 0xA0 && temp_char <= 0xF9) {
                    index = temp_char - 0xA0;
                    init_level = 2;
                } else if (temp_char != 0xFA) {
                    init_level = 0;
                }
            } else if (init_level == 2) {

                // speed
                uint16_t motor_speed_;
                boost::asio::read(serial_, boost::asio::buffer(&speed_, 2));
                scan.time_increment = (1 / (((uint16_t) ((speed_[1] << 8) + speed_[0]) >> 6) / 60.0)) / 360.0;
                scan.scan_time = (1 / (((uint16_t) ((speed_[1] << 8) + speed_[0]) >> 6)) / 60.0);

                //ROS_ERROR("Speed 1 : %d", (((speed_[1]<<8) + speed_[0]) >> 6));
                //ROS_ERROR("Speed 2 : %d", ((((speed_[1]<<8) + speed_[0]) >> 6) / 60));
                //ROS_ERROR("Speed 3 : %f", scan.time_increment);

                // data
                boost::asio::read(serial_, boost::asio::buffer(&raw_bytes_, 16));

                uint8_t byte0 = raw_bytes_[0];
                uint8_t byte1 = raw_bytes_[1];
                uint8_t byte2 = raw_bytes_[2];
                uint8_t byte3 = raw_bytes_[3];
                // First two bits of byte1 are status flags
                uint8_t flag1 = (byte1 & 0x80) >> 7;       // No return/max range/too low of reflectivity
                uint8_t flag2 = (byte1 & 0x40) >> 6;       // Object too close, possible poor reading due to proximity kicks in at < 0.6m
                // Remaining bits are the range in mm
                uint16_t range = 0;
                if (flag1 == 0)
                    range = ((byte1 & 0x3F) << 8) + byte0;
                else
                    range = UNSEEN_VALUE;
                // Last two bytes represent the uncertanty or intensity, might also be pixel area of target...
                uint16_t intensity = (byte3 << 8) + byte2;

                scan.ranges[index * 4 + 0] = (range / 1000.0);
                scan.intensities[index * 4 + 0] = intensity;

                byte0 = raw_bytes_[4];
                byte1 = raw_bytes_[5];
                byte2 = raw_bytes_[6];
                byte3 = raw_bytes_[7];
                // First two bits of byte1 are status flags
                flag1 = (byte1 & 0x80) >> 7;               // No return/max range/too low of reflectivity
                flag2 = (byte1 & 0x40) >> 6;               // Object too close, possible poor reading due to proximity kicks in at < 0.6m
                // Remaining bits are the range in mm
                if (flag1 == 0)
                    range = ((byte1 & 0x3F) << 8) + byte0;
                else
                    range = UNSEEN_VALUE;
                // Last two bytes represent the uncertanty or intensity, might also be pixel area of target...
                intensity = (byte3 << 8) + byte2;

                scan.ranges[index * 4 + 1] = (range / 1000.0);
                scan.intensities[index * 4 + 1] = intensity;

                byte0 = raw_bytes_[8];
                byte1 = raw_bytes_[9];
                byte2 = raw_bytes_[10];
                byte3 = raw_bytes_[11];
                // First two bits of byte1 are status flags
                flag1 = (byte1 & 0x80) >> 7;               // No return/max range/too low of reflectivity
                flag2 = (byte1 & 0x40) >> 6;               // Object too close, possible poor reading due to proximity kicks in at < 0.6m
                // Remaining bits are the range in mm
                if (flag1 == 0)
                    range = ((byte1 & 0x3F) << 8) + byte0;
                else
                    range = UNSEEN_VALUE;
                // Last two bytes represent the uncertanty or intensity, might also be pixel area of target...
                intensity = (byte3 << 8) + byte2;

                scan.ranges[index * 4 + 2] = (range / 1000.0);
                scan.intensities[index * 4 + 2] = intensity;

                byte0 = raw_bytes_[12];
                byte1 = raw_bytes_[13];
                byte2 = raw_bytes_[14];
                byte3 = raw_bytes_[15];
                // First two bits of byte1 are status flags
                flag1 = (byte1 & 0x80) >> 7;               // No return/max range/too low of reflectivity
                flag2 = (byte1 & 0x40) >> 6;               // Object too close, possible poor reading due to proximity kicks in at < 0.6m
                // Remaining bits are the range in mm
                if (flag1 == 0)
                    range = ((byte1 & 0x3F) << 8) + byte0;
                else
                    range = UNSEEN_VALUE;
                // Last two bytes represent the uncertanty or intensity, might also be pixel area of target...
                intensity = (byte3 << 8) + byte2;

                scan.ranges[index * 4 + 3] = (range / 1000.0);
                scan.intensities[index * 4 + 3] = intensity;

                boost::array < uint8_t, 2 > checksum_;
                boost::asio::read(serial_, boost::asio::buffer(&checksum_, 2));

                init_level = 0;
            }
            if (index == 89){
		scan.header.stamp = ros::Time::now() - ros::Duration(0.137); // 0.2 for laser print ; 0.1 for obstacle detection
                if(abs(tspeed.data) < 2.0) {
			laser_pub.publish(scan);
		//	ROS_ERROR("        Publishing");
		}
		else {
		//	ROS_ERROR("! NOT ! Publishing");
		}
		//broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.1)),
                //                                       scan.header.stamp, "base_link", "neato_laser"));
	    }
        }




    }
    catch(boost::system::system_error ex) {
        ROS_ERROR("Error instantiating laser object. Are you sure you have the correct port and baud rate? Error was %s", ex.what());
        return -1;
    }
}

