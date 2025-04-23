#include "ros/ros.h"
#include "std_msgs/String.h"
#include <math.h>
#include <tf/transform_broadcaster.h>
#include <numeric>  
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>

#include <first_project/sector_times.h>  //Filter

// Globals from your original code
double lat, lon, alt;
ros::Time current_time, last_time;
double lat_ref, lon_ref, alt_ref;
double V = 0.0;  // Speed
double steering_angle;

std::deque<double> speeds; // Use deque for efficient front removal

ros::Publisher gps_odom_pub;
tf::TransformBroadcaster* gps_odom_broadcaster = nullptr;

// ===== Sector Tracking Globals =====
struct Waypoint {
  double lat;
  double lon;
};

std::vector<Waypoint> sector_points = {
  {45.616031, 9.280769}, // Sector 1 start
  {45.630108, 9.289489}, // Sector 2 start
  {45.623572, 9.287294}, // Sector 3 start
};

int current_sector = 1;
ros::Time last_sector_time;
double sector_distance = 0.0;

ros::Publisher sector_pub;

Waypoint last_position;
bool has_last_position = false;


bool entered_first_sector = false;


double haversine(double lat1, double lon1, double lat2, double lon2) {
  constexpr double R = 6371000.0;
  double dLat = (lat2 - lat1) * M_PI / 180.0;
  double dLon = (lon2 - lon1) * M_PI / 180.0;
  lat1 = lat1 * M_PI / 180.0;
  lat2 = lat2 * M_PI / 180.0;
  
  double a = sin(dLat / 2) * sin(dLat / 2) +
  sin(dLon / 2) * sin(dLon / 2) * cos(lat1) * cos(lat2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return R * c;
}

int getCurrentSector(const Waypoint& pos) {
  size_t next = current_sector % sector_points.size();
  Waypoint next_wp = sector_points[next];

  double d = haversine(pos.lat, pos.lon, next_wp.lat, next_wp.lon);
  
  // Consider the vehicle has entered the next sector if it's within 20 meters
  if (d < 20.0) {
      current_sector = next + 1; // Move to next sector

      // Wrap around to sector 1 when it exceeds the last sector
      if (current_sector > sector_points.size()) {
          current_sector = 1; // Go back to sector 1
      }

      last_sector_time = current_time;
      sector_distance = 0.0;
      ROS_INFO_STREAM("Entered sector " << current_sector);
  }

  return current_sector;
}
// ====== Callbacks ======

void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg){
  lat = msg->latitude;
  lon = msg->longitude;
  alt = msg->altitude;
  current_time = msg->header.stamp;
  
  Waypoint current_pos = {lat, lon};

  if (has_last_position) {
    double dist = haversine(last_position.lat, last_position.lon, lat, lon);
    sector_distance += dist;
  }

  int new_sector = getCurrentSector(current_pos);

  // First-ever entry into Sector 1
  if (!entered_first_sector && new_sector == 1) {
    last_sector_time = current_time;
    entered_first_sector = true;
    ROS_INFO("Initial entry into Sector 1 - timer started");
  }

  // Subsequent transitions (including wrapping from 3 → 1)
  if (entered_first_sector && new_sector != current_sector) {
    current_sector = new_sector;
    last_sector_time = current_time;
    sector_distance = 0.0;
    speeds.clear(); // reset speed samples
    ROS_INFO_STREAM("Entered sector " << current_sector);
  }

  last_position = current_pos;
  has_last_position = true;
}




double getAverageSpeed() {
  double avg_speed = 0.0;
  
  // If there are speed values in the deque, calculate the average
  if (!speeds.empty()) {
    double sum = std::accumulate(speeds.begin(), speeds.end(), 0.0);
    avg_speed = sum / speeds.size();
  }
  
  return avg_speed;
}

void speedCallback(const geometry_msgs::PointStamped msg){
  steering_angle = msg.point.x;
  
  // Only accept reasonable values (filter out noise/spikes)
  if (msg.point.y > 0.5 && msg.point.y < 100.0) {
    speeds.push_back(msg.point.y); // Add new speed value
  } else {
    speeds.push_back(0.0);  // Ignore invalid speed values, or mark as zero if vehicle is stopped
  }
  
  // Keep only the last 20 speed readings
  if (speeds.size() > 20) {
    speeds.pop_front();  // Remove the oldest value
  }
  
  current_time = msg.header.stamp;
}
int main(int argc, char **argv){
  
  ros::init(argc, argv, "gps_odometer");
  ros::NodeHandle n;
  
  gps_odom_pub = n.advertise<nav_msgs::Odometry>("gps_odom", 50);
  sector_pub = n.advertise<first_project::sector_times>("sector_times", 10);
  tf::TransformBroadcaster _gps_odom_broadcaster;
  gps_odom_broadcaster = &_gps_odom_broadcaster;
  
  current_time = ros::Time::now(); 
  last_time = ros::Time::now();
  last_sector_time = ros::Time::now();
  
  n.getParam("lat_r", lat_ref);
  n.getParam("lon_r", lon_ref);
  n.getParam("alt_r", alt_ref);
  
  ros::Subscriber gps_sub = n.subscribe("swiftnav/front/gps_pose", 100, gpsCallback);
  ros::Subscriber speed_sub = n.subscribe("speedsteer", 100, speedCallback);
  
  ros::Rate r(10);
  while (n.ok()) {
    ros::spinOnce();
    
    // Publish sector info at 10Hz
    ros::Duration elapsed = current_time - last_sector_time;
    first_project::sector_times msg;
    msg.current_sector = current_sector;
    msg.current_sector_time = elapsed.toSec();
    msg.current_sector_mean_speed = getAverageSpeed();
    
    
    
    sector_pub.publish(msg);
    
    last_time = current_time;
    r.sleep();
  }
}
