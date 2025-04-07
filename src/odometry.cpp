#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <math.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

//Variables
double steering_angle = 0;
double speed_front = 0;

double dt;
double fr_distance; //Distance between front and rear wheels 
double R; //Distance to the CIR
double V; //Linear speed of the vehicle
double w; //Angular velocity of the vehicle

double x = 0; // Initial x-position
double y = 0; // Initial y-position
double theta = 0; // Initial angle

double vx;
double vy;

ros::Time current_time;
ros::Time last_time;

nav_msgs::Odometry odom;
ros::Publisher odom_pub;
tf::TransformBroadcaster *odom_broadcaster;

// Methods

void computeOdometry(){

    //A. We obtain the linear speed of the vehicle (V)
    dt = (current_time - last_time).toSec();


    // Check if the steering angle is valid (not too small or zero)
    if (steering_angle == 0) {
        // If the steering angle is zero, we are driving straight, set radius to a large value
        R = std::numeric_limits<double>::infinity();
    } else {
        // A1. Radius of Curvature (R) for Ackermann steering
        R = fr_distance / tan(steering_angle * (M_PI / 180));  // fr_distance is the wheelbase length (L)
    }
    
    // Ensure radius is not zero or infinite (check for invalid situations)
    if (R == 0 || std::isinf(R)) {
        ROS_WARN("Invalid radius, steering angle too small or zero. Skipping update.");
        return;  // Skip update to prevent NaN propagation
    }

    w = (((speed_front * 1000)/3600)*sin((steering_angle)*(M_PI/180)))/fr_distance;

    V = w*R;

    //B. We integrate to estimate the angle of roation and then the position
    vx = V * cos(theta);
    vy = V * sin(theta);
    
    theta += w * dt;
    x += vx * dt;
    y += vy * dt;
}


void publishOdom(){
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link"; 
    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
        
    //send the transform
    odom_broadcaster->sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //Set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
    
    //Set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = w;

    odom_pub.publish(odom);
}


void subscriptionCallback(const geometry_msgs::PointStamped msg){
    steering_angle = msg.point.x;
    speed_front = msg.point.y;
  }


int main(int argc, char **argv){
    /* 1. Odometry node initialization */
    ros::init(argc, argv, "odometry");
    ros::NodeHandle n;
    odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    tf::TransformBroadcaster _odom_broadcaster;
    odom_broadcaster = &_odom_broadcaster;
   
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    n.getParam("/fr_distance", fr_distance);

    /* 2. Internal subscriber configuration */
    ros::Subscriber sub = n.subscribe("speedsteer", 100, subscriptionCallback);
    ros::spinOnce();

    ros::Rate r(1.0);
    while(n.ok()){
        ros::spinOnce(); //We obtain the raw information by listening to speedsteer
        current_time = ros::Time::now();

        /* 3. We compute the odometry */
        computeOdometry();

        /* 4. We publish the odometry and tf */
        publishOdom();

        ROS_INFO("I heard: [%f, %f, %f]", x, y, theta);

        last_time = current_time;
        //r.sleep();
    }
}