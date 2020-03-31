#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int8.h"
#include <sstream>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>

int main(int argc, char **argv)
{
 ros::init(argc, argv, "gps_kvh_dummy_node");
 /**
  * NodeHandle is the main access point to communications with the ROS system.
  * The first NodeHandle constructed will fully initialize this node, and the last NodeHandle destructed will close down the node.
  */
 ros::NodeHandle n;  /**
  * The advertise() function is how you tell ROS that you want to publish on a given topic name. This invokes a call to the ROS
  * master node, which keeps a registry of who is publishing and who is subscribing. The second parameter to advertise() is the size of the message queue.
  */
 ros::Publisher chatter_pub_st = n.advertise<std_msgs::String>("gps/utmzone", 100);
 ros::Publisher chatter_pub_x = n.advertise<std_msgs::Float64>("gps/utmx", 100);
 ros::Publisher chatter_pub_y = n.advertise<std_msgs::Float64>("gps/utmy", 100);
 ros::Publisher chatter_pub_i8 = n.advertise<std_msgs::Int8>("gps/kvhstatus", 100);
 ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 100);
 ros::Publisher nav_fix_pub = n.advertise<sensor_msgs::NavSatFix>("gps/fix", 100);
 ros::Publisher nav_stat_pub = n.advertise<sensor_msgs::NavSatStatus>("gps/status", 100);
 tf::TransformBroadcaster odom_broadcaster;
 double x = 5283561.467821;
 double y = 696056.755074;
 double th = 0.0;
 double vx = 0.1;
 double vy = -0.1;
 double vth = 0.1;
 double lat = 47.675682;
 double lon = 17.609613;

 ros::Time current_time, last_time;
 current_time = ros::Time::now();
 last_time = ros::Time::now();
 ros::Rate loop_rate(10);
 ROS_INFO("GSP dummy started");

 while (ros::ok())
 {
   std_msgs::String msg_utmzone;
   std_msgs::Float64 msg_utmx;
   std_msgs::Float64 msg_utmy;
   std_msgs::Int8 msg_kvhstatus;
   msg_utmzone.data = std::to_string(33) + "T"; // 33T is west hungary, 34T is east hungary, 30U is london
   msg_utmx.data = x / 1000;
   msg_utmy.data = y / 1000;
   msg_kvhstatus.data = 0; // 0 is ok
   /**
    * The publish() function is how you send messages. The parameter
    * is the message object. The type of this object must agree with the type
    * given as a template parameter to the advertise<>() call, as was done
    * in the constructor above.
    */
   current_time = ros::Time::now();

   //compute odometry in a typical way given the velocities of the robot
   double dt = (current_time - last_time).toSec();
   double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
   double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
   double delta_th = vth * dt;

   x += delta_x;
   y += delta_y;
   th += delta_th;
   lat += (delta_x / 100);
   lon += (delta_y / 100);


   // since all odometry is 6DOF we'll need a quaternion created from yaw
   geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

   //  transform over tf
   geometry_msgs::TransformStamped odom_trans;
   odom_trans.header.stamp = current_time;
   odom_trans.header.frame_id = "odom";
   odom_trans.child_frame_id = "base_link";
   odom_trans.transform.rotation = odom_quat;

   // send the transform
   odom_broadcaster.sendTransform(odom_trans);

   // odometry message over ROS
   nav_msgs::Odometry odom;
   odom.header.stamp = current_time;
   odom.header.frame_id = "odom";

   // set the position
   odom.pose.pose.position.x = x; // UTM Position Packet X e.g. 5283561.467821
   odom.pose.pose.position.y = y; // UTM Position Packet Y e.g. 696056.836622
   odom.pose.pose.position.z = 157.0; // UTM Position Packet e.g. 157 m height
   odom.pose.pose.orientation.x = 0.862203; // Quaternion Orientation Packet QX
   odom.pose.pose.orientation.y = 0.006738; // Quaternion Orientation Packet QY
   odom.pose.pose.orientation.z = -0.50650; // Quaternion Orientation Packet QZ
   odom.pose.pose.orientation.w = -0.00356; // Quaternion Orientation Packet QS, not w


   // nav fix message over ROS
   sensor_msgs::NavSatFix fix;
   fix.header.stamp = current_time;
   fix.latitude = lat;
   fix.longitude = lon;

   // nav staus message over ROS
   sensor_msgs::NavSatStatus nstatus;
   nstatus.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
   nstatus.status = 0;

   // publish the messages
   odom_pub.publish(odom);
   nav_fix_pub.publish(fix);
   nav_stat_pub.publish(nstatus);


   chatter_pub_st.publish(msg_utmzone);
   chatter_pub_x.publish(msg_utmx);
   chatter_pub_y.publish(msg_utmy);
   chatter_pub_i8.publish(msg_kvhstatus);
   // ROS_INFO("%.2f %.2f", x, y);
   ros::spinOnce();
   last_time = current_time;
   loop_rate.sleep();
 }
 return 0;
}
