/*
 * Translates sensor_msgs/NavSat{Fix,Status} into nav_msgs/Odometry using UTM
 */

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/NavSatFix.h>
#include <gps_common/conversions.h>
#include <nav_msgs/Odometry.h>
#include <multi_car_msgs/GPS.h>
#include <boost/lexical_cast.hpp>

using namespace gps_common;

ros::Publisher * odom_pub;
ros::Subscriber * fix_sub;
std::string frame_id, child_frame_id;
double rot_cov;
int Ncars;

void callback(const multi_car_msgs::GPSPtr& gps) {
  sensor_msgs::NavSatFix fix;
  int car_id;
  int roomba_id = gps->car_id;
  std::string id_dict = "/id_dict/";
  std::string roomba_str = boost::lexical_cast<std::string>(roomba_id);
  std::string dict_key = id_dict + roomba_str;

  ros::param::get(dict_key, car_id);

  // if (roomba_id == 0 || roomba_id == 26677 || roomba_id == 26626){
  //   car_id = 0;
  // }
  // else if (roomba_id == 1 || roomba_id == 26727 || roomba_id == 26630){
  //   car_id = 1;
  // }
  // else if (roomba_id == 2 || roomba_id == 26715 || roomba_id == 26663){
  //   car_id = 2;
  // }
  fix = gps->fix;
  std::string car = "car";
  std::string int_id = boost::lexical_cast<std::string>(car_id);
  std::string new_frame_id = car + int_id; 
  if (fix.status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX) {
    ROS_INFO("No fix.");
    return;
  }

  if (gps->header.stamp == ros::Time(0)) {
    return;
  }

  double northing, easting;
  std::string zone;

  LLtoUTM(fix.latitude, fix.longitude, northing, easting, zone);


  if (odom_pub[car_id]) {

    nav_msgs::Odometry odom;
    odom.header.stamp = gps->header.stamp;

    if (frame_id.empty())
      odom.header.frame_id = gps->header.frame_id;
    else
      odom.header.frame_id = frame_id;

    odom.child_frame_id = child_frame_id;

    odom.pose.pose.position.x = easting;
    odom.pose.pose.position.y = northing;
    odom.pose.pose.position.z = fix.altitude;
    
    odom.pose.pose.orientation.x = 0;
    odom.pose.pose.orientation.y = 0;
    odom.pose.pose.orientation.z = 0;
    odom.pose.pose.orientation.w = 1;
    
    // Use ENU covariance to build XYZRPY covariance
    boost::array<double, 36> covariance = {{
      fix.position_covariance[0],
      fix.position_covariance[1],
      fix.position_covariance[2],
      0, 0, 0,
      fix.position_covariance[3],
      fix.position_covariance[4],
      fix.position_covariance[5],
      0, 0, 0,
      fix.position_covariance[6],
      fix.position_covariance[7],
      fix.position_covariance[8],
      0, 0, 0,
      0, 0, 0, rot_cov, 0, 0,
      0, 0, 0, 0, rot_cov, 0,
      0, 0, 0, 0, 0, rot_cov
    }};

    odom.pose.covariance = covariance;

    odom_pub[car_id].publish(odom);
  }
}

int main (int argc, char **argv) {
  ros::init(argc, argv, "utm_odometry_node");
  ros::NodeHandle node;
  ros::NodeHandle priv_node("~");

  priv_node.param<int>("num_cars", Ncars, 3);
  priv_node.param<std::string>("frame_id", frame_id, "");
  priv_node.param<std::string>("child_frame_id", child_frame_id, "");
  priv_node.param<double>("rot_covariance", rot_cov, 99999.0);

  odom_pub = new ros::Publisher[Ncars];
  fix_sub = new ros::Subscriber[Ncars];

  for(int i=0; i<Ncars; i++){
    std::string int_id = boost::lexical_cast<std::string>(i);
    odom_pub[i] = node.advertise<nav_msgs::Odometry>("odom" + int_id, 10);
    std::string bullshit = "car" + int_id;
    if (!frame_id.compare(bullshit)){
      fix_sub[i] = node.subscribe("/car" + int_id + "/fix", 10, callback);
    }
    else{
      fix_sub[i] = node.subscribe("/fix", 10, callback);
    }
  }

  ros::spin();
}

