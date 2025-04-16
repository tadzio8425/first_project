#include "ros/ros.h"
#include "std_msgs/String.h"
#include <math.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>




void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg){  
    lat = msg->latitude;
    lon = msg->longitude;
    alt = msg->altitude;
    current_time = msg->header.stamp;
  }

  void speedCallback(const geometry_msgs::PointStamped msg){
    steering_angle = msg.point.x;
    V = msg.point.y;
    current_time = msg.header.stamp;
  }

int main(int argc, char **argv){
    /* 1. GPS Odometry node initialization */
    ros::init(argc, argv, "gps_odometer");
    ros::NodeHandle n;
    gps_odom_pub = n.advertise<nav_msgs::Odometry>("gps_odom", 50);
    tf::TransformBroadcaster _gps_odom_broadcaster;
    gps_odom_broadcaster = &_gps_odom_broadcaster;
   
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    n.getParam("lat_r", lat_ref);
    n.getParam("lon_r", lon_ref);
    n.getParam("alt_r", alt_ref);

    /* 2. Internal subscribers configuration */
    /* Subscriber to gps_pose*/
    ros::Subscriber sub = n.subscribe("swiftnav/front/gps_pose", 100, gpsCallback);
    /* Subscriber to speedtester*/
    ros::Subscriber sub = n.subscribe("speedsteer", 100, speedCallback);

    ros::Rate r(10);
    while(n.ok()){
        ros::spinOnce(); //We obtain the raw information by listening to speedsteer

        /* 3. We compute the gps to odometry */
        gps_to_odom(lat_ref, lon_ref, alt_ref);

        /* 4. We publish the odometry and tf */
        publish_gps_Odom();

        last_time = current_time;
        r.sleep();
    }
}