#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>
#include <math.h>
#include <iostream>
#include <exception>
#include <string>
#include <GeographicLib/UTMUPS.hpp>
#include <GeographicLib/MGRS.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;
using namespace GeographicLib;
using namespace std;

#define PI 3.1415926535897932846
double gnss_2_utm_station_x = 0.0;
double gnss_2_utm_station_y = 0.0;
int gnss_2_utm_epsg = 0;

double model_car_x = 0.0;
double model_car_y = 0.0;
double model_car_z = 0.0;
size_t count_=0;

class Gnss_2_Utm_Node : public rclcpp::Node
{
public:
  Gnss_2_Utm_Node()
  : Node("gnss_2_utm")
  {
    pose_cov_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/sensing/gnss/pose_with_covariance",10,std::bind(&Gnss_2_Utm_Node::pose_cov_callback, this, _1));
    pose_2d_pub = this->create_publisher<geometry_msgs::msg::Pose2D>("Truck_RTKPose2D", 1);    // CHANGE
  }

  void pose_cov_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose_cov_msg) const       // CHANGE
  {
    geometry_msgs::msg::Pose2D pose_2d;
    string GZD_ID = "51RUP";
    string mgrs = GZD_ID + to_string(static_cast<int64_t>(pose_cov_msg->pose.pose.position.x*100000.0)) + to_string(static_cast<int64_t>(pose_cov_msg->pose.pose.position.y*100000.0));//"51RUP40568030735840851086"
    int zone, prec;
    bool northp;
    double x, y;
    MGRS::Reverse(mgrs, zone, northp, x, y, prec);
    double lat, lon;
    UTMUPS::Reverse(zone, northp, x, y, lat, lon);
    pose_2d.x = lat;
    pose_2d.y = lon;

    tf2::Quaternion q(pose_cov_msg->pose.pose.orientation.x,
                      pose_cov_msg->pose.pose.orientation.y,
                      pose_cov_msg->pose.pose.orientation.z,
                      pose_cov_msg->pose.pose.orientation.w);
    tf2::Matrix3x3 matrix(q);
    double roll,pitch,yaw;
    double theta;
    matrix.getRPY(roll,pitch,yaw);
    
    theta = -1 * (yaw - PI / 2) * 180.0 / PI;
    if(theta < 0.0)
    {
      theta = theta + 360.0;
    }
    else if(theta >= 360.0)
    {
      theta = 360.0 - theta;
    }
    pose_2d.theta = theta;
    pose_2d_pub->publish(pose_2d);	
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_cov_sub;
  rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pose_2d_pub;         // CHANGE
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  RCLCPP_INFO(rclcpp::get_logger("gnss_2_utm"), "Log In Gnss_2_Utm Success!!!!!!!");    // CHANGE
  rclcpp::spin(std::make_shared<Gnss_2_Utm_Node>());
  rclcpp::shutdown();
  return 0;
}
