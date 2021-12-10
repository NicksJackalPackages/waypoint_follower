#include <ros/ros.h>
#include <ros/console.h>
#include <vector>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/Path.h"
#include "waypoint_follower_msgs/WaypointArray.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <robot_localization/navsat_conversions.h>

/* Written by Nick Sullivan, The University of Adelaide.

*/

using std::vector;
using std::cout;
using std::endl;
using std::string;
using geometry_msgs::PointStamped;
using geometry_msgs::PoseStamped;
using geometry_msgs::TransformStamped;
using geometry_msgs::Twist;
//using waypoint_follower_msgs::Waypoint;
//using waypoint_follower_msgs::WaypointStamped;
using waypoint_follower_msgs::WaypointArray;

WaypointArray waypoint_array;          //list of waypoints to complete
int current_waypoint_index = -1;       //the index in waypoint_array
int id_counter = 0;                    //for generating ID's
double lin_vel;                        //forward speed
double ang_vel;                        //rotate speed
double start_delay;                    //time to wait before going to waypoints
double dist_req;                       //distance required to each waypoint for success
string name_space;                     //prestring for frame ID's
string map_frame;                      //"map"
string odom_frame;                     //"name_space/odom"
string baselink_frame;                 //"name_space/base_link"
string launch_frame;				           //frame_id of launch file waypoints
bool include_start_pose;               //include the start pose in path visualisation
bool enabled;

PoseStamped start_pose;

ros::Publisher pub_cmd_vel;
ros::Publisher pub_waypoint_paths;
//ros::Publisher pub_completed_paths;
ros::Subscriber sub_set_waypoints;

tf2_ros::Buffer tfbuffer;            //for transformation lookup
tf::TransformListener* listener;

/**************************************************
 * Helper functions
 **************************************************/

// Convert GPS coordinates into values relative to the UTM frame.
PointStamped convertLatLongToUTM(double lati, double longi){
  //cout << "Converting: ( " << lati << ", " << longi << ")" << endl;
  double utm_northing, utm_easting;
  string utm_zone;
  RobotLocalization::NavsatConversions::LLtoUTM(lati, longi, utm_northing, utm_easting, utm_zone);
  //cout << cout.precision(10) << "Results: ( " << utm_easting << ", " << utm_northing << ")" << endl;
  //cout << "zone: " << utm_zone << endl;
  geometry_msgs::PointStamped ps;
  ps.header.frame_id = "utm";
  ps.header.stamp = ros::Time(0);
  ps.point.x = utm_easting;
  ps.point.y = utm_northing;
  return ps;
}

// This is similar to 'convertPoseFrame', but for a special case.
bool convertUTMtoMap( geometry_msgs::PointStamped &p ){
  try {
    geometry_msgs::PointStamped old_p, new_p;
    ros::Time now         = ros::Time(0);
    string frame_in       = "utm";
    string frame_out      = map_frame;
    old_p.header.frame_id = frame_in;
    old_p.header.stamp    = now;
    old_p.point = p.point;
    tf::TransformListener listener;
    listener.waitForTransform(frame_in, frame_out, now, ros::Duration(10) );
    listener.transformPoint(frame_out, old_p, new_p);
    new_p.point.z = 0;
    p = new_p;
  } catch (tf2::TransformException e){
    cout << "Failed to convert UTM to map coordinates";
    return false;
  }
}

// Given a pose belonging to frame_in, changes it to if it were from frame_out.
// Returns whether it was successful.
bool convertPoseFrame( geometry_msgs::Pose &p, string frame_in, string frame_out ){
  try {
    
    cout << "Time is: " << ros::Time::now() << endl;
    cout << "Looking up transform from " << frame_in << " to " << frame_out << endl;
    geometry_msgs::PoseStamped old_pose, new_pose;
    ros::Time now            = ros::Time(0);
    //ros::Time now            = ros::Time::now();
    old_pose.pose            = p;
    old_pose.header.frame_id = frame_in;
    old_pose.header.stamp    = now;

    listener->waitForTransform(frame_in, frame_out, now, ros::Duration(20) );
    listener->transformPose(frame_out, old_pose, new_pose);

    p = new_pose.pose;

    return true;
  } catch (tf2::TransformException e){
    cout << "Failed to lookup transform from " << frame_in << " to " << frame_out << endl;
    return false;
  }
}

// Looks up a transform from frame1 to frame2 using tf2. Returns whether the lookup
// was successful.
bool lookupTransform(TransformStamped &output, string frame1, string frame2, ros::Time t){
  try{
    //cout << "Trying to get from frame: " << frame1 << " to " << frame2 << endl;
    if (!tfbuffer.canTransform(frame1, frame2, t) ){
      ROS_WARN("Unable to obtain a TF in observing.cpp");
      return false;
    }
    output = tfbuffer.lookupTransform(frame1, frame2, t);
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    return false;
  }
  return true;
}


/**************************************************
 * Publisher functions
 **************************************************/

void publishWaypointPaths(){
  //cout << "Publishing waypoint paths!" << endl;
  nav_msgs::Path p;
  p.header.seq      = 0;
  p.header.stamp    = ros::Time::now();
  p.header.frame_id = map_frame;
  vector<geometry_msgs::PoseStamped> vec;
  geometry_msgs::PoseStamped ps;
  // Add robot start position.
  if( include_start_pose ){
    geometry_msgs::Pose pose;
    pose.orientation.w = 1;
    convertPoseFrame( pose, baselink_frame, map_frame );
    ps.header.stamp = ros::Time::now();
    ps.header.frame_id = baselink_frame;
    ps.pose = pose;
    vec.push_back( ps );
  }
  // Add waypoints.
  for( int i=current_waypoint_index; i<waypoint_array.waypoints.size(); i++ ){
    if( i < 0 ) continue;
    PoseStamped ws = waypoint_array.waypoints.at(i);
    ps.header = ws.header;
    ps.pose   = ws.pose;
    if( ws.header.frame_id != map_frame ){
      convertPoseFrame( ps.pose, ws.header.frame_id, map_frame );
    }
    ps.header.frame_id = map_frame;
    vec.push_back( ps );
  }
  p.poses = vec;
  pub_waypoint_paths.publish( p );
}

 /**************************************************
 * Subscriber functions
 **************************************************/

void callbackSetWaypoints(const WaypointArray::ConstPtr& msg){
  cout << "Received waypoints!" << endl;
  waypoint_array = *msg;
  if (current_waypoint_index < 1) {
    enabled = true;
  }
}

/**************************************************
 * Looping
 **************************************************/
void publishTimerCallback(const ros::TimerEvent& e){
    publishWaypointPaths();
}
// Called repeatedly until all waypoints are completed.
void loop(){
  cout << "*******************" << endl;
  if( !enabled || current_waypoint_index < 0) return;
  double x, y, wp_x, wp_y, wp_ang, qx, qy, qz, qw, roll, pitch, yaw, dx, dy, dist, ang;

  // Grab our current location.
  cout << "Getting current location. " << endl;
  TransformStamped ts_map2base;
  if ( !lookupTransform(ts_map2base, map_frame, baselink_frame, ros::Time(0)) ) return;
  x  = ts_map2base.transform.translation.x;
  y  = ts_map2base.transform.translation.y;
  qx = ts_map2base.transform.rotation.x;
  qy = ts_map2base.transform.rotation.y;
  qz = ts_map2base.transform.rotation.z;
  qw = ts_map2base.transform.rotation.w;
  tf2::Quaternion quat(qx, qy, qz, qw);
  tf2::Matrix3x3 mat(quat);
  mat.getRPY(roll, pitch, yaw);
  cout << "My location: ( " << x << " , " << y << " )" << endl;
  //cout << "x, y, yaw: (" << x << " , " << y << " , " << yaw << ")" << endl;

  // Grab the waypoint location.
  //cout << "current_waypoint_index: " << current_waypoint_index << endl;
  PoseStamped ps = waypoint_array.waypoints[current_waypoint_index];
  // Calculate the distance from our location to our target location.
  //cout << "Getting destination waypoint. " << endl;
  wp_x = ps.pose.position.x;
  wp_y = ps.pose.position.y;
  dx   = wp_x - x;
  dy   = wp_y - y;
  wp_ang = atan2( dy, dx );
  cout << "Destination: ( " << wp_x << " , " << wp_y << " )" << endl;
  //cout << "x, y, yaw: (" << wp_x << " , " << wp_y <<  " , " << wp_ang << ")" << endl;

  // Convert from x-y to dist-ang.
  dist = sqrt( dx*dx + dy*dy );
  ang  = wp_ang - yaw;
  if( ang >  3.1415 ) ang = ang - 2*3.1415;
  if( ang < -3.1415 ) ang = ang + 2*3.1415;
  //cout << "dx, dy: (" << dx << " , " << dy << ")" << endl;
  cout << "Target dist, ang: (" << dist << " , " << ang*180/3.1415 << ")" << endl;

  // Produce a command velocity.
  double x_vel, yaw_vel;
  if( dist > dist_req ) {
    x_vel = lin_vel;
  } else {
    cout << " Completed a waypoint. Current index is " << current_waypoint_index << endl;
    cout << " Total waypoints are: " << waypoint_array.waypoints.size() << endl;
    //ros::Duration(15).sleep();
    current_waypoint_index++;
    if( current_waypoint_index >= waypoint_array.waypoints.size() ){
      x_vel   = 0;
      enabled = false;
      cout << "WAYPOINTS DONE" << endl;
    }
  }
  yaw_vel = 0;
  double ang_abs = sqrt( ang*ang );
  if( ang_abs > 0.5 ) {                //yaw speed cap
    yaw_vel = ang_vel;
  } else {
    yaw_vel = ang_vel*ang_abs / 0.5;   //linear
  }
  if( ang < 0 ){
    yaw_vel = -yaw_vel;
  }
  Twist cmd_vel;
  cmd_vel.linear.x  = x_vel;
  cmd_vel.angular.z = yaw_vel;
  cout << "xlin, yawvel: (" << x_vel << " , " << yaw_vel << ")" << endl;
  pub_cmd_vel.publish(cmd_vel);
}

/**************************************************
 * Initialising functions
 **************************************************/

void loadSubs(ros::NodeHandle n){
  //sub_add_waypoint     = n.subscribe("add_waypoint",      100, callbackAddWaypoint);
  sub_set_waypoints    = n.subscribe("set_waypoints",     100, callbackSetWaypoints);
  //sub_upd_waypoints    = n.subscribe("update_waypoints",  100, callbackUpdateWaypoints);
  //sub_obs_waypoint     = n.subscribe("observed_waypoints",100, callbackObservedWaypoints);
  //sub_move_base_result = n.subscribe("move_base/result",  100, callbackMoveBaseResult);
}

void loadPubs(ros::NodeHandle n){
  pub_cmd_vel         = n.advertise<geometry_msgs::Twist>("jackal_velocity_controller/cmd_vel", 10);
  pub_waypoint_paths  = n.advertise<nav_msgs::Path>("waypoint_paths", 10);
  //pub_completed_paths = n.advertise<nav_msgs::Path>("completed_paths", 10);
}

// Loads parameters from the launch file. Uses default values if any values are
// missing.
void loadParams(ros::NodeHandle n_priv){
  vector<double> waypoint_values;    //position (x,y,z) and orientation (x,y,z,w)
  vector<double> gps_values;         //position (lat, long)
  vector<int>    waypoint_order;     //indices of waypoint_values
  vector<int>    gps_waypoint_order; //indices of gps_values
  enabled = false;
  // Set default parameters.
  double default_waypoint_values[] = {};
  double default_gps_values[]      = {};
  string default_baselink_frame    = "base_link";
  string default_map_frame         = "map";
  string default_odom_frame        = "odom";
  double default_lin_vel           = 0.5;
  double default_ang_vel           = 0.5;
  double default_start_delay       = 0;
  bool default_include_start_pose  = true;
  double default_dist_req          = 1;

  n_priv.param("baselink_frame",     baselink_frame,     default_baselink_frame);
  n_priv.param("map_frame",          map_frame,          default_map_frame);
  n_priv.param("odom_frame",         odom_frame,         default_odom_frame);
  n_priv.param("launch_frame",       launch_frame,       odom_frame);
  n_priv.param("include_start_pose", include_start_pose, default_include_start_pose);
  n_priv.param("lin_vel",            lin_vel,            default_lin_vel);
  n_priv.param("ang_vel",            ang_vel,            default_ang_vel);
  n_priv.param("start_delay",        start_delay,        default_start_delay);
  n_priv.param("dist_req",           dist_req,           default_dist_req);

  if( start_delay > 0 ){
    ros::Duration(start_delay).sleep();
  }

  // Get waypoint order.
  XmlRpc::XmlRpcValue v;
  if( n_priv.getParam("waypoint_order", v) ){
    ROS_ASSERT(v.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for(int i=0; i < v.size(); i++)  {
      if( v[i].getType() == XmlRpc::XmlRpcValue::TypeDouble ) {
        waypoint_order.push_back(v[i]);
      }
      if( v[i].getType() == XmlRpc::XmlRpcValue::TypeInt ) {
        int d = v[i];
        waypoint_order.push_back(d);
      }
    }
  }
  // Get waypoints.
  if( n_priv.getParam("waypoints", v) ){
    ROS_ASSERT(v.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for(int i=0; i < v.size(); i++)  {
      if( v[i].getType() == XmlRpc::XmlRpcValue::TypeDouble ) {
        waypoint_values.push_back(v[i]);
      }
      if( v[i].getType() == XmlRpc::XmlRpcValue::TypeInt ) {
        int d = v[i];
        waypoint_values.push_back(d);
      }
    }
  } else {
    waypoint_values.assign(default_waypoint_values, default_waypoint_values +
                      sizeof(default_waypoint_values) / sizeof(default_waypoint_values[0]) );
  }

  // Convert waypoint values into waypoints.
  if( waypoint_values.size() % 2 != 0 ){
    cout << "INCORRECT NUMBER OF WAYPOINT VALUES" << endl;
    return;
  }
  // Fill in order (starts with 1).
  if( waypoint_order.empty() ){
    while( waypoint_order.size() < waypoint_values.size()/2 ){
      waypoint_order.push_back( waypoint_order.size()+1 );
    }
  }
  for( int i=0; i<waypoint_order.size(); i++ ) {
    int j = (waypoint_order[i]-1)*2;
    PoseStamped ps;
    ps.header.seq      = 0;
    ps.header.stamp    = ros::Time::now();
    ps.header.frame_id = launch_frame;
    ps.pose.position.x    = waypoint_values[j];
    ps.pose.position.y    = waypoint_values[j+1];
    ps.pose.orientation.w = 1;
    waypoint_array.waypoints.push_back(ps);
    cout << "wp: " << ps.pose.position.x << " , " << ps.pose.position.y << endl;
    enabled = true;
  }

  // Get GPS order.
  if( n_priv.getParam("gps_waypoint_order", v) ){
    ROS_ASSERT(v.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for(int i=0; i < v.size(); i++)  {
      if( v[i].getType() == XmlRpc::XmlRpcValue::TypeDouble ) {
        gps_waypoint_order.push_back(v[i]);
      }
      if( v[i].getType() == XmlRpc::XmlRpcValue::TypeInt ) {
        int d = v[i];
        gps_waypoint_order.push_back(d);
      }
    }
  }
  // Get GPS locations.
  if( n_priv.getParam("gps_waypoints", v) ){
    ROS_ASSERT(v.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for(int i=0; i < v.size(); i++)  {
      if( v[i].getType() == XmlRpc::XmlRpcValue::TypeDouble ) {
        gps_values.push_back(v[i]);
      }
      if( v[i].getType() == XmlRpc::XmlRpcValue::TypeInt ) {
        int d = v[i];
        gps_values.push_back(d);
      }
    }
  } else {
    gps_values.assign(default_gps_values, default_gps_values +
                      sizeof(default_gps_values) / sizeof(default_gps_values[0]) );
  }
  // Convert gps values into waypoints, store them relative to the map_frame.
  if( gps_values.size() % 2 != 0 ){
    cout << "INCORRECT NUMBER OF GPS WAYPOINT VALUES" << endl;
    return;
  }
  // Fill in default order (starts with 1).
  if( gps_waypoint_order.empty() ){
    while( gps_waypoint_order.size() < gps_values.size()/2 ){
      gps_waypoint_order.push_back( gps_waypoint_order.size()+1 );
    }
  }
  cout << "NUMBER OF GPS WAYPOINTS: " << gps_values.size()/2 << endl;
  for( int i=0; i<gps_waypoint_order.size(); i++ ) {
    int j = (gps_waypoint_order[i]-1)*2;
    cout << "FIRST GPS PAIR LOADED" << endl;
    PoseStamped ws;
    double lati  = gps_values[j];
    double longi = gps_values[j+1];
    PointStamped ps = convertLatLongToUTM(lati, longi);
    convertUTMtoMap( ps );

    ws.header = ps.header;
    ws.pose.position.x = ps.point.x;
    ws.pose.position.y = ps.point.y;
    ws.pose.position.z = ps.point.z;
    ws.pose.orientation.x = 0;
    ws.pose.orientation.y = 0;
    ws.pose.orientation.z = 0;
    ws.pose.orientation.w = 1;
    waypoint_array.waypoints.push_back(ws);
    enabled = true;
  }
}


int main(int argc, char** argv){
  ros::init(argc, argv, "waypoint_follower");
  ros::NodeHandle n;
  ros::NodeHandle n_priv("~");
  loadSubs(n);
  loadPubs(n);
  loadParams(n_priv);
  tf2_ros::TransformListener tflistener(tfbuffer);   //must stay persistent
  tf::TransformListener listen;
  listener = &listen;

  current_waypoint_index = 0;
  ros::Timer timer = n.createTimer(ros::Duration(1), publishTimerCallback);

  ros::Rate loop_rate(10);
  while( ros::ok() ){
    loop();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
};


