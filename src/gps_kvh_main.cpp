#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int8.h"
#include <sstream>
#include <iostream>
#include <string>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include "Rs232.hpp"
#include "AnPacketProtocol.hpp"
#include "SpatialPackets.hpp"
//#include "GpsFix.hpp"

enum GPS_PUB_FLAG {NORMAL, FIX_HEIGHT};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "gps_kvh_node");
	ros::NodeHandle nh_private("~");
	std::string port_param;
	double fix_height = -10000;
	GPS_PUB_FLAG gps_pub_flag = NORMAL;
	int baud_rate;
	if (!nh_private.getParam("port", port_param)||!nh_private.getParam("baud", baud_rate))
	{
		ROS_WARN("Usage e.g.: rosrun gps_kvh gps_kvh_node /dev/ttyUSB0 115200");
		ROS_WARN("Also make sure /dev/ttyUSB0 or /dev/ttyS0 has permissions >> chmod");
		ROS_ERROR("GPS did not started");
		exit(EXIT_FAILURE);
	}
	if (nh_private.getParam("fix_height", fix_height))
	{
		gps_pub_flag = FIX_HEIGHT;
		ROS_WARN("Setting fix height: %f", fix_height);
	}
	else
	{
		ROS_INFO("Using height from GPS");
	}
	
	/*
	if (argc != 3)
	{
		ROS_WARN("Usage e.g.: rosrun gps_kvh gps_kvh_node /dev/ttyUSB0 115200");
		ROS_WARN("Also make sure /dev/ttyUSB0 or /dev/ttyS0 has permissions >> chmod");
		ROS_ERROR("GPS did not started");
		exit(EXIT_FAILURE);
	}
	*/
	//if (OpenComport(argv[1], atoi(argv[2])))
	if (OpenComport(port_param.c_str(), baud_rate))
	{
		ROS_ERROR("Could not open serial port");
	}
	an_decoder_t an_decoder;
	an_decoder_initialise(&an_decoder);
	an_packet_t *an_packet;
	system_state_packet_t system_state_packet;
	raw_sensors_packet_t raw_sensors_packet;
	utm_position_packet_t utm_position_packet;
	geodetic_position_packet_t geodetic_position_packet;
	status_packet_t status_packet;
	unix_time_packet_t unix_time_packet;
	quaternion_orientation_packet_t quaternion_orientation_packet;
	satellites_packet_t satellites_packet;
	int bytes_received;
	ros::NodeHandle n;
	ros::Publisher utmZonePub = n.advertise<std_msgs::String>("gps/utmzone", 10);
	ros::Publisher chatter_pub_st = n.advertise<std_msgs::String>("gps/utmzone", 10);
	ros::Publisher chatter_pub_i8 = n.advertise<std_msgs::Int8>("gps/kvhstatus", 10);
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("gps/odom", 10);
	ros::Publisher nav_fix_pub = n.advertise<sensor_msgs::NavSatFix>("gps/fix", 10);
	ros::Publisher nav_stat_pub = n.advertise<sensor_msgs::NavSatStatus>("gps/status", 10);
	ros::Publisher current_pose_pub = n.advertise<geometry_msgs::PoseStamped>("gps/current_pose", 10);
	ros::Publisher gnss_fix_pub_i8 = n.advertise<std_msgs::Int8>("gps/fixstatus", 10);
	ros::Time current_time;
	//tf::TransformBroadcaster odom_broadcaster;
	ros::Rate loopRate(50);
	ROS_INFO("GPS node started (50Hz)");
	while (ros::ok())
	{

		if ((bytes_received = PollComport(an_decoder_pointer(&an_decoder), an_decoder_size(&an_decoder))) > 0)
		{
			current_time = ros::Time::now();
			bool odom_utm_ok = false;
			bool odom_quat_ok = false;
			std_msgs::String msg_utmzone;
			std_msgs::Int8 msg_kvhstatus;
			std_msgs::Int8 msg_fixstatus;
			nav_msgs::Odometry odom;
			sensor_msgs::NavSatFix fix;
			sensor_msgs::NavSatStatus nstatus;
			geometry_msgs::PoseStamped current_pose;
			an_decoder_increment(&an_decoder, bytes_received);
			nstatus.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
			nstatus.status = 0;
			odom.header.stamp = current_time;
			odom.child_frame_id = "gps";
			odom.header.frame_id = "world";
			current_pose.header.stamp = current_time;
			current_pose.header.frame_id = "world";
			//geometry_msgs::TransformStamped odom_trans;
			//odom_trans.header.stamp = current_time;
			//odom_trans.child_frame_id = "gps";
			//odom_trans.header.frame_id = "map";
			//odom_broadcaster.sendTransform(odom_trans);

			while ((an_packet = an_packet_decode(&an_decoder)) != NULL) // decode
			{
				//ROS_WARN("an_packet->id %d ", an_packet->id);
				if (an_packet->id == packet_id_system_state) // system state packet
				{
					if (decode_system_state_packet(&system_state_packet, an_packet) == 0)
					{
						//fix.header.stamp = current_time;
						//fix.latitude = system_state_packet.latitude * RADIANS_TO_DEGREES;
						//fix.longitude = system_state_packet.longitude * RADIANS_TO_DEGREES;
						//fix.altitude = system_state_packet.height;
						msg_fixstatus.data = system_state_packet.filter_status.b.gnss_fix_type;
						gnss_fix_pub_i8.publish(msg_fixstatus);
						//ROS_INFO("Gps fix: %d", msg_fixstatus.data);
						// 0 - No GNSS fix - no RTK
						// 1 - 2D GNSS fix - no RTK
						// 2 - 3D GNSS fix - no RTK
						// 3 - SBAS GNSS fix - no RTK
						// 4 - Differential GNSS fix - no RTK
						// 5 - Omnistar/Starfire GNSS fix - no RTK
						// 6 - RTK Float GNSS fix
						// 7 - RTK Fixed GNSS fix
					}
				}
				else if (an_packet->id == packet_id_status) // status_packet
				{
					if (decode_status_packet(&status_packet, an_packet) == 0)
					{
						int kvh_status = status_packet.system_status.b.system_failure +
										 status_packet.system_status.b.accelerometer_sensor_failure +
										 status_packet.system_status.b.gyroscope_sensor_failure +
										 status_packet.system_status.b.magnetometer_sensor_failure +
										 status_packet.system_status.b.pressure_sensor_failure +
										 status_packet.system_status.b.gnss_failure +
										 status_packet.system_status.b.accelerometer_over_range +
										 status_packet.system_status.b.gyroscope_over_range +
										 status_packet.system_status.b.magnetometer_over_range +
										 status_packet.system_status.b.pressure_over_range +
										 status_packet.system_status.b.minimum_temperature_alarm +
										 status_packet.system_status.b.maximum_temperature_alarm +
										 status_packet.system_status.b.low_voltage_alarm +
										 status_packet.system_status.b.high_voltage_alarm +
										 status_packet.system_status.b.gnss_antenna_disconnected +
										 status_packet.system_status.b.serial_port_overflow_alarm;
						msg_kvhstatus.data = kvh_status; // sum of all error, so 0 is ok
						chatter_pub_i8.publish(msg_kvhstatus);
					}
					else
						ROS_INFO("Status_Decode failed\n");
				}

				else if (an_packet->id == packet_id_utm_position) // utm_position_packet
				{
					if (decode_utm_position_packet(&utm_position_packet, an_packet) == 0)
					{
						odom.pose.pose.position.x = utm_position_packet.position[1]; // Position Packet X Easting (m)
						odom.pose.pose.position.y = utm_position_packet.position[0]; // Position Packet Y Northing (m)
						switch (gps_pub_flag){
							case NORMAL:{
								odom.pose.pose.position.z = utm_position_packet.position[2];
								current_pose.pose.position.z = utm_position_packet.position[2];
								break;
							}
							case FIX_HEIGHT:{
								current_pose.pose.position.z = fix_height;
								odom.pose.pose.position.z = fix_height; // Position Packet e.g. 157 m height
								break;
							}
						}
						//odom.pose.pose.position.z = utm_position_packet.position[2];
						msg_utmzone.data = std::to_string(utm_position_packet.zone_number) + utm_position_packet.zone_character; // 33T is west hungary, 34T is east hungary, 30U is london
						chatter_pub_st.publish(msg_utmzone);
						odom_utm_ok = true;
						current_pose.pose.position.x = utm_position_packet.position[1]; // X Easting
						current_pose.pose.position.y = utm_position_packet.position[0]; // Y Northing
						//current_pose.pose.position.z = utm_position_packet.position[2];
						
						//odom_trans.transform.translation.x = utm_position_packet.position[1]; // X Easting
						//odom_trans.transform.translation.y = utm_position_packet.position[0]; // Y Easting
						//odom_trans.transform.translation.z = utm_position_packet.position[2];
					}
					else
						ROS_INFO("UTM_Decode failed\n");
				}


				else if (an_packet->id == packet_id_geodetic_position) // geodetic_position_packet
				{
					if (decode_geodetic_position_packet(&geodetic_position_packet, an_packet) == 0)
					{
						fix.header.stamp = current_time;
						fix.latitude = geodetic_position_packet.position[0] * RADIANS_TO_DEGREES;
						fix.longitude = geodetic_position_packet.position[1] * RADIANS_TO_DEGREES;
						fix.altitude = geodetic_position_packet.position[2];
						nav_fix_pub.publish(fix);
					}
					else
						ROS_INFO("Geodetic_Decode failed\n");
				}
				else if (an_packet->id == packet_id_quaternion_orientation) // quaternion_orientation_packet
				{

					if (decode_quaternion_orientation_packet(&quaternion_orientation_packet, an_packet) == 0)
					{
						tf2::Quaternion tf_orig(quaternion_orientation_packet.orientation[1], 
							quaternion_orientation_packet.orientation[2], 
							-quaternion_orientation_packet.orientation[3], 
							quaternion_orientation_packet.orientation[0]);
						tf2::Quaternion tf_rot, tf_aligned;
						tf_rot.setRPY(0.0, 0.0, M_PI_2);
						tf_aligned = tf_rot*tf_orig;

						odom.pose.pose.orientation.x = tf_aligned.x();								 // Quaternion Orientation Packet QX
						odom.pose.pose.orientation.y = tf_aligned.y();								 // Quaternion Orientation Packet QY
						odom.pose.pose.orientation.z = tf_aligned.z();								 // Quaternion Orientation Packet QZ
						odom.pose.pose.orientation.w = tf_aligned.w();								 // Quaternion Orientation Packet QS, or w
						odom_quat_ok = true;
						current_pose.pose.orientation.x = tf_aligned.x();
						current_pose.pose.orientation.y = tf_aligned.y();
						current_pose.pose.orientation.z = tf_aligned.z();
						current_pose.pose.orientation.w = tf_aligned.w();

						//odom_trans.transform.rotation.x = tf_aligned.x();
						//odom_trans.transform.rotation.y = tf_aligned.y();
						//odom_trans.transform.rotation.z = tf_aligned.z();
						//odom_trans.transform.rotation.w = tf_aligned.w();
					}
					else
						ROS_INFO("Quaternion_Decode failed\n");
				}

				else if (an_packet->id == packet_id_satellites) // satellites_packet
				{

					if (decode_satellites_packet(&satellites_packet, an_packet) == 0)
					{
						int sum_satelites = satellites_packet.gps_satellites + satellites_packet.glonass_satellites + satellites_packet.beidou_satellites + satellites_packet.galileo_satellites + satellites_packet.sbas_satellites;
						nstatus.status = sum_satelites;
						nav_stat_pub.publish(nstatus);
						//ROS_INFO("sum_satelites %d", sum_satelites);
					}
					else
						ROS_INFO("Satellites decode failed\n");
				}
				//else if(an_packet->id == packet_id_raw_sensors) // raw sensors packet
				//{
				//	if(decode_raw_sensors_packet(&raw_sensors_packet, an_packet) == 0)
				//	{
				//		printf("Raw Sensors Packet:\n");
				//		printf("\tAccelerometers X: %f Y: %f Z: %f\n", raw_sensors_packet.accelerometers[0], raw_sensors_packet.accelerometers[1], raw_sensors_packet.accelerometers[2]);
				//		printf("\tGyroscopes X: %f Y: %f Z: %f\n", raw_sensors_packet.gyroscopes[0] * RADIANS_TO_DEGREES, raw_sensors_packet.gyroscopes[1] * RADIANS_TO_DEGREES, raw_sensors_packet.gyroscopes[2] * RADIANS_TO_DEGREES);
				//	}
				//}
				//else if(an_packet->id == packet_id_unix_time) // unix_time_packet
				//{
				//	if(decode_unix_time_packet(&unix_time_packet, an_packet) == 0)
				//	{
				//		printf("Unix Time Packet:\n");
				//		printf("\tSecond: %u Microsecond: %u\n", unix_time_packet.unix_time_seconds,
				//							unix_time_packet.microseconds);
				//	}
				//	else
				//		ROS_INFO("Unix_Decode failed\n");
				//}
				//else
				//{
				//	printf("Packet ID %u of Length %u\n", an_packet->id, an_packet->length);
				//}

				// Ensure that you free the an_packet when your done with it or you will leak memory
				an_packet_free(&an_packet);
			}

			// publish odom
			if (odom_utm_ok and odom_quat_ok){
				odom_pub.publish(odom);
				current_pose_pub.publish(current_pose);
				//odom_broadcaster.sendTransform(odom_trans);
				odom_utm_ok = odom_quat_ok = false;
			}
			ros::spinOnce();
			loopRate.sleep();
		}
	}
	return 0;
}
