#include "ros/ros.h"
#include "std_msgs/String.h"
#include <math.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>


const double a = 6378137.0;           
const double b = 6356752.0;
const double e2 = 1 - (pow(b, 2) / pow(a, 2));

//variables
double x_ECEF, y_ECEF, z_ECEF;
double x, y, z, N;
ros::Time current_time, last_time;
double lat_ref, lon_ref, alt_ref;
double lat, lon, alt;

nav_msgs::Odometry gps_odom;
ros::Publisher gps_odom_pub;
tf::TransformBroadcaster *gps_odom_broadcaster;


void gps_to_odom(double lat_ref, double lon_ref, double alt_ref){

    // Conversion to radians (Math functions are in radians)
    double lat_rad = lat * M_PI / 180.0;
    double lon_rad = lon * M_PI / 180.0;
    double lat0_rad = lat_ref * M_PI / 180.0;
    double lon0_rad = lon_ref * M_PI / 180.0;

    // ECEF conversion of reference point
    double N0 = a / sqrt(1 - e2 * pow(sin(lat0_rad), 2));
    double x0 = (N0 + alt_ref) * cos(lat0_rad) * cos(lon0_rad);
    double y0 = (N0 + alt_ref) * cos(lat0_rad) * sin(lon0_rad);
    double z0 = (N0 * (1 - e2) + alt_ref) * sin(lat0_rad);

    // ECEF conversion of current point
    N = a / sqrt(1 - e2 * pow(sin(lat_rad), 2));
    x_ECEF = (N + alt) * cos(lat_rad) * cos(lon_rad);
    y_ECEF = (N + alt) * cos(lat_rad) * sin(lon_rad);
    z_ECEF = (N * (1 - e2) + alt) * sin(lat_rad);

    // Xp - Xr
    double dx = x_ECEF - x0;
    double dy = y_ECEF- y0;
    double dz = z_ECEF- z0;

    // ENU conversion
    
    x = -sin(lon0_rad) * dx + cos(lon0_rad) * dy;
    y = -sin(lat0_rad) * cos(lon0_rad) * dx
          - sin(lat0_rad) * sin(lon0_rad) * dy
          + cos(lat0_rad) * dz;
    z =  cos(lat0_rad) * cos(lon0_rad) * dx
          + cos(lat0_rad) * sin(lon0_rad) * dy
          + sin(lat0_rad) * dz;

}


void publish_gps_Odom() {
    // Publish gps_odometry
    gps_odom.header.stamp = current_time;
    gps_odom.header.frame_id = "odom_gps";
    gps_odom.child_frame_id = "gps";

    gps_odom.pose.pose.position.x = x;
    gps_odom.pose.pose.position.y = y;
    gps_odom.pose.pose.position.z = z;

    gps_odom.pose.pose.orientation.x = 0.0; //still needing work, missing heading calculation
    gps_odom.pose.pose.orientation.y = 0.0;
    gps_odom.pose.pose.orientation.z = 0.0;
    gps_odom.pose.pose.orientation.w = 1.0; 

    gps_odom_pub.publish(gps_odom);

    
    geometry_msgs::TransformStamped gps_odom_tf;
    gps_odom_tf.header.stamp = current_time;
    gps_odom_tf.header.frame_id = "odom-gps";
    gps_odom_tf.child_frame_id = "gps";

    gps_odom_tf.transform.translation.x = x;
    gps_odom_tf.transform.translation.y = y;
    gps_odom_tf.transform.translation.z = z;

    gps_odom_tf.transform.rotation.x = 0.0;
    gps_odom_tf.transform.rotation.y = 0.0;
    gps_odom_tf.transform.rotation.z = 0.0;
    gps_odom_tf_tf.transform.rotation.w = 1.0;
    // send transform
    gps_odom_broadcaster->sendTransform(gps_odom_tf);
}

void subscriptionCallback(const sensor_msgs::NavSatFix msg){
    lat = msg->latitude;
    lon = msg->longitude;
    alt = msg->altitude;
    current_time = msg->header.stamp;
  }


int main(int argc, char **argv){
    /* 1. GPS Odometry node initialization */
    ros::init(argc, argv, "gps_odometer");
    ros::NodeHandle n;
    ros::Publisher gps_odom_pub = n.advertise<nav_msgs::Odometry>("gps_odom", 50);
    tf::TransformBroadcaster _gps_odom_broadcaster;
    gps_odom_broadcaster = &_gps_odom_broadcaster;
   
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    n.getParam("lat_r", lat_ref);
    n.getParam("lon_r", lon_ref);
    n.getParam("alt_r", alt_ref);

    /* 2. Internal subscriber configuration */
    ros::Subscriber sub = n.subscribe("swiftnav/front/gps_pose", 100, subscriptionCallback);

    ros::Rate r(10000);
    while(n.ok()){
        ros::spinOnce(); //We obtain the raw information by listening to speedsteer

        /* 3. We compute the gps to odometry */
        gps_to_odom(lat_ref, lon_ref, alt_ref);

        /* 4. We publish the odometry and tf */
        publish_gps_Odom();

        ROS_INFO("I heard: [%f, %f, %f]", x, y, z);

        last_time = current_time;
        r.sleep();
    }
}