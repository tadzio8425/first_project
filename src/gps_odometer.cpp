#include "ros/ros.h"
#include "std_msgs/String.h"
#include <math.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>

const double a = 6378137.0;           
const double b = 6356752.0;
const double e2 = 1 - (pow(b, 2) / pow(a, 2));

struct Point {
    double x, y;
};


//variables
double x_ECEF, y_ECEF, z_ECEF;
double x, y, z, N;
ros::Time current_time, last_time;
double lat_ref = 45.618932;
double lon_ref = 9.281179;
double alt_ref = 229.049061;
double theta;
double lat, lon, alt;

nav_msgs::Odometry gps_odom;
ros::Publisher gps_odom_pub;
tf::TransformBroadcaster *gps_odom_broadcaster;


// Heading window - Config
const size_t window_size = 20;
std::deque<Point> window;
Point p;




// Function to compute smoothed heading using a moving average window
double computeSmoothedHeading(const std::deque<Point>& positions) {
    static double last_valid_heading = 0.0; // memory for fallback
    if (positions.size() < 2) return last_valid_heading;

    double sum_dx = 0.0;
    double sum_dy = 0.0;
    const double movement_threshold = 0.01;
    bool valid_movement_found = false;

    for (size_t i = 1; i < positions.size(); ++i) {
        double dx = positions[i].x - positions[i - 1].x;
        double dy = positions[i].y - positions[i - 1].y;
        double dist = std::hypot(dx, dy);

        if (dist < movement_threshold) continue; // skip noisy steps

        sum_dx += dx;
        sum_dy += dy;
        valid_movement_found = true;
    }

    if (!valid_movement_found) {
        return last_valid_heading; // return last known heading if nothing valid
    }

    double heading = std::atan2(sum_dy, sum_dx);
    last_valid_heading = heading; // update fallback
    return heading;
}

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


    p.x = x;
    p.y = y;

    window.push_back(p);
    if (window.size() > window_size) {
        window.pop_front();
    }

    if (window.size() >= 2) {
        theta = computeSmoothedHeading(window);
    }
    

}


void publish_gps_Odom() {

    geometry_msgs::TransformStamped gps_odom_tf;
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);
    gps_odom_tf.header.stamp = current_time;
    gps_odom_tf.header.frame_id = "odom";
    gps_odom_tf.child_frame_id = "gps";
    

    gps_odom_tf.transform.translation.x = x;
    gps_odom_tf.transform.translation.y = y;
    gps_odom_tf.transform.translation.z = z;

    gps_odom_tf.transform.rotation = odom_quat;
    // send transform
    gps_odom_broadcaster->sendTransform(gps_odom_tf);


    // Publish gps_odometry
    gps_odom.header.stamp = current_time;
    gps_odom.header.frame_id = "odom";
    gps_odom.child_frame_id = "gps";

    gps_odom.pose.pose.position.x = x;
    gps_odom.pose.pose.position.y = y;
    gps_odom.pose.pose.position.z = z;

    gps_odom.pose.pose.orientation = odom_quat;
    

    gps_odom_pub.publish(gps_odom);

    

}

void subscriptionCallback(const sensor_msgs::NavSatFix::ConstPtr& msg){
    lat = msg->latitude;
    lon = msg->longitude;
    alt = msg->altitude;
    current_time = msg->header.stamp;
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

    /* 2. Internal subscriber configuration */
    ros::Subscriber sub = n.subscribe("swiftnav/front/gps_pose", 100, subscriptionCallback);

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