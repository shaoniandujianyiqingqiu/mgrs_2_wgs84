#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
//#include "eco_ros_lib/msg/num.hpp"     // CHANGE
//#include "eco_ros_lib/msg/gpfpd.hpp"     // CHANGE
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

// geometry_msgs::msg::Point llh_2_xyz(double longitude,double latitude,double altitude)
// {

// 	/******************wsg84_2_Web Mercator*********************/
//   if(gnss_2_utm_epsg==3857)
//   {
//     geometry_msgs::msg::Point utm_pose_;
//     double x = longitude*20037508.34/180.0;
//     double y = log(tan((90.0+latitude)*PI/360.0))/(PI/180.0);
//     double z = altitude;
// 	  y = y * 20037508.34 / 180;
// 	  utm_pose_.z = 0.0;  //utm_pose_.z = hig;
// 	  utm_pose_.x = x - gnss_2_utm_station_x;
// 	  utm_pose_.y = y - gnss_2_utm_station_y;
//     return utm_pose_; 
//   }

// 	/************  wsg84_2_utm ************/
//   else if(gnss_2_utm_epsg==32650)
//   {
//     geometry_msgs::msg::Point utm_pose_;
//     double lon = longitude;
//     double lat = latitude;
//     double hig = altitude;
//     // variable
//     double a = 6378.137;
//     double e = 0.0818192;
//     double k0 = 0.9996;
//     double E0 = 500;
//     double N0 = 0;
//     //calc zoneNumber
//     double zoneNumber = floor(lon/6) + 31;
//     //calc lambda0
//     double lambda0 = (zoneNumber - 1) * 6 - 180 + 3; //deg
//     lambda0 = lambda0 * PI / 180.0; //radian
//     //calc phi and lambda (lat and lon)
//     double phi = lat * PI / 180.0;
//     double lambda = lon * PI / 180.0;
//     // Formula START
//     double v = 1 / sqrt(1 - pow(e*sin(phi), 2));
//     double A = (lambda - lambda0) * cos(phi);
//     double T = pow(tan(phi), 2);
//     double C = pow(e, 2) / (1 - pow(e, 2)) * pow(cos(phi), 2);
//     double s = (1 - pow(e, 2)/4 - 3*pow(e, 4)/64 - 5*pow(e, 6)/256)*phi - (3*pow(e, 2)/8 + 3*pow(e, 4)/32 + 45*pow(e, 6)/1024)*sin(2*phi) + (15*pow(e, 4)/256 + 45*pow(e, 6)/1024)*sin(4*phi) - 35*pow(e, 6)/3072*sin(6*phi);
//     utm_pose_.x = E0 + k0*a*v * (A + (1-T+C)*pow(A, 3)/6 + (5-18*T+T*T)*pow(A, 5)/120);
//     utm_pose_.y = N0 + k0*a * (s + v*tan(phi) * (pow(A, 2)/2 + (5-T+9*C+4*C*C)*pow(A, 4)/24 + (61-58*T+T*T)*pow(A, 6)/720));
//     utm_pose_.z = 0.0;  //utm_pose_.z = hig;

//     utm_pose_.x *= 1000.0;
//     utm_pose_.y *= 1000.0;
//     utm_pose_.x = utm_pose_.x - gnss_2_utm_station_x;
//     utm_pose_.y = utm_pose_.y - gnss_2_utm_station_y;
//     return utm_pose_;
//   }
//   else
//   {
//     //ROS_INFO("\033[1;31m----> Error Do Not Get gnss_2_utm_epsg !!!!!!.\033[0m");
//     RCLCPP_ERROR(rclcpp::get_logger("gnss_2_utm"), "Error Do Not Get gnss_2_utm_epsg !!!!!!");    // CHANGE	
//   }
// }

geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0,0,yaw);
  return tf2::toMsg(q);
}

class Gnss_2_Utm_Node : public rclcpp::Node
{
public:
  Gnss_2_Utm_Node()
  : Node("gnss_2_utm")
  {
    //pose_sub = this->create_subscription<sensor_msgs::msg::NavSatFix>("/sensing/gnss/ublox/nav_sat_fix",10,std::bind(&Gnss_2_Utm_Node::pose_callback, this, _1));
    pose_cov_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/sensing/gnss/pose_with_covariance",10,std::bind(&Gnss_2_Utm_Node::pose_cov_callback, this, _1));
    pose_2d_pub = this->create_publisher<geometry_msgs::msg::Pose2D>("Truck_RTKPose2D", 1);    // CHANGE
  }

// void pose_callback(const sensor_msgs::msg::NavSatFix::SharedPtr pose_msg) const       // CHANGE
//   {
    
//      double lat, lon;
//      lat = pose_msg->latitude;
//      lon = pose_msg->longitude;
//       int zone;
//       bool northp;
//       double x, y;
//       UTMUPS::Forward(lat, lon, zone, northp, x, y);
//       cout << "lat:"<<lat << "\n";
//       cout << "lon:"<<lon << "\n";
//       cout << "zone:"<<zone << "\n";
//       cout << "northp:"<<northp << "\n";
//       cout << "x:"<<x << "\n";
//       cout << "y:"<<y << "\n";
//       string mgrs;
//       MGRS::Forward(zone, northp, x, y, lat, 10, mgrs);
//       cout << "mgrs:"<<mgrs << "\n";


//       //string mgrsq = "51RUP40568030735840851086";
//       // int zoneq, precq;
//       // bool northpq;
//       // double xq, yq;
//       // MGRS::Reverse(mgrs, zoneq, northpq, xq, yq, precq);
//       // //cout << "mgrsq:"<<mgrsq << "\n";
//       // cout << "zoneq:"<<zoneq << "\n";
//       // cout << "northpq:"<<northpq << "\n";
//       // cout << "northp:"<<northp << "\n";
//       // cout << "xq:"<<x << "\n";
//       // cout << "y:"<<y << "\n";
//       // double latq, lonq;
//       // UTMUPS::Reverse(zoneq, northpq, xq, yq, latq, lonq);
//       // cout << precq << " " << latq << " " << lonq << "\n";


//             string aa ,bb;
//       aa = mgrs.substr(5,10);
//       bb = mgrs.substr(15,24);
//       cout << "aa:"<<aa << "\n";
//       cout << "bb:"<<bb << "\n";

//       char *aax = (char*)aa.c_str();
//       char *bby = (char*)bb.c_str();
//       double ax = atof(aax);
//       double bx = atof(bby);
//       ax = ax * 0.00001;
//       bx = bx * 0.00001;
//       cout << "ax:"<<ax << "\n";
//       cout << "bx:"<<bx << "\n";

//     geometry_msgs::msg::Pose2D pose_2d;
    
//     pose_2d.theta = 0.0;
//     pose_2d.x = ax;
//     pose_2d.y = bx;

//     pose_2d_pub->publish(pose_2d);	

    

//   }
 
 
  void pose_cov_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose_cov_msg) const       // CHANGE
  {
    geometry_msgs::msg::Pose2D pose_2d;
    string GZD_ID = "51RUP";
    string mgrs = GZD_ID + to_string(static_cast<int64_t>(pose_cov_msg->pose.pose.position.x*100000.0)) + to_string(static_cast<int64_t>(pose_cov_msg->pose.pose.position.y*100000.0));//"51RUP40568030735840851086"
    cout << "mgrs:"<<mgrs << "\n";
    int zone, prec;
    bool northp;
    double x, y;
    MGRS::Reverse(mgrs, zone, northp, x, y, prec);
    double lat, lon;
    UTMUPS::Reverse(zone, northp, x, y, lat, lon);
    cout << prec << " " << lat << " " << lon << "\n";
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
    cout << "theta:"<<theta << "\n";

    //double heading_ = -1 * gnss_msg->heading * PI / 180.0 + PI / 2;


		//car_pose.pose.orientation = createQuaternionMsgFromYaw(heading_);
    // RCLCPP_INFO(this->get_logger(), "longitude: %lf", gnss_msg->longitude); 
    // RCLCPP_INFO(this->get_logger(), "latitude: %lf", gnss_msg->latitude); 

		 //RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", message.num); 
    


    pose_2d_pub->publish(pose_2d);	

    

  }

private:
  //rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr pose_sub;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_cov_sub;
  rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pose_2d_pub;         // CHANGE
  
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  RCLCPP_INFO(rclcpp::get_logger("gnss_2_utm"), "Log In Gnss_2_Utm Success!!!!!!!");    // CHANGE

  

//       double lat = 30.347050, lon = 121.341271; // latitude:  30.347050 longitude:  121.341271
//                                                 //			x:  40564.677311		y:  58410.117261
//                                                 // 51RUP40568030735840851086
//       int zone;
//       bool northp;
//       double x, y;
//       UTMUPS::Forward(lat, lon, zone, northp, x, y);
//       cout << "lat:"<<lat << "\n";
//       cout << "lon:"<<lon << "\n";
//       cout << "zone:"<<zone << "\n";
//       cout << "northp:"<<northp << "\n";
//       cout << "x:"<<x << "\n";
//       cout << "y:"<<y << "\n";
//       string mgrs;
//       MGRS::Forward(zone, northp, x, y, lat, 10, mgrs);
//       cout << "mgrs:"<<mgrs << "\n";
//       string aa ,bb;
//       aa = mgrs.substr(5,10);
//       bb = mgrs.substr(15,24);
//       cout << "aa:"<<aa << "\n";
//       cout << "bb:"<<bb << "\n";

//       char *aax = (char*)aa.c_str();
//       char *bby = (char*)bb.c_str();
//       double ax = atof(aax);
//       double bx = atof(bby);
//       ax = ax * 0.00001;
//       bx = bx * 0.00001;
//       cout << "ax:"<<ax << "\n";
//       cout << "bx:"<<bx << "\n";
//       constexpr int GZD_ID_size = 5;
//       double xxx,yyy;
//     xxx = std::stod(mgrs.substr(GZD_ID_size, 10)) *std::pow(10, -5);  // set unit as [m]
//     yyy = std::stod(mgrs.substr(GZD_ID_size + 10, 10)) *std::pow(10, -5);  // set unit as [m]
// //cout << mgrs_code << "\n";
// cout << xxx << "\n";
// cout << yyy << "\n";


// //**************************
//   int zoneg;
//   bool northpg;
//   double xg, yg;
//   double xxx,yyy;
//   string mgrs_code;
//     GeographicLib::UTMUPS::Forward(lat, lon, zoneg, northpg, xg, yg);
    
//   GeographicLib::MGRS::Forward(
//       zoneg, northpg, xg, yg, lat, 9, mgrs_code);
//       constexpr int GZD_ID_size = 5;
      
//     xxx = std::stod(mgrs_code.substr(GZD_ID_size, 9)) *std::pow(10, 5 -9);  // set unit as [m]
//     yyy = std::stod(mgrs_code.substr(GZD_ID_size + 9, 9)) *std::pow(10, 5 -9);  // set unit as [m]
// cout << mgrs_code << "\n";
// cout << xxx << "\n";
// cout << yyy << "\n";
   
//       // Sample reverse calculation
//       string mgrsq = "51RUP40568030735840851086";
//       int zoneq, precq;
//       bool northpq;
//       double xq, yq;
//       MGRS::Reverse(mgrsq, zoneq, northpq, xq, yq, precq);
//       cout << "mgrsq:"<<mgrsq << "\n";
//       cout << "zoneq:"<<zoneq << "\n";
//       cout << "northpq:"<<northpq << "\n";
//       cout << "northp:"<<northp << "\n";
//       cout << "xq:"<<x << "\n";
//       cout << "y:"<<y << "\n";
//       double latq, lonq;
//       UTMUPS::Reverse(zoneq, northpq, xq, yq, latq, lonq);
//       cout << precq << " " << latq << " " << lonq << "\n";
      //207.865,30.34791,121.341504,,,,,,,
//,,,0.515706,0,0,-0.856765,40593.64793,58506.77051,11.663
 
  rclcpp::spin(std::make_shared<Gnss_2_Utm_Node>());
  rclcpp::shutdown();
  return 0;
}
