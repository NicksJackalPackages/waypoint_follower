#include <ros/ros.h>
#include <ros/console.h>
#include <vector>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/Path.h"
#include "waypoint_follower_msgs/Waypoint.h"
#include "waypoint_follower_msgs/WaypointStamped.h"
#include "waypoint_follower_msgs/WaypointArray.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <robot_localization/navsat_conversions.h>

/* Written by Nick Sullivan, The University of Adelaide. 
   This file takes a series of waypoints and periodically sends them to 
   move_base. The waypoints can come from a launch file, or dynamically added.
   Also supports feedback i.e. if the waypoint is observed in a new position,
   the waypoint is updated and re-sent to move_base. 
   The inputs can come from three sources, from a launch file, from RVIZ, or 
   from any node producing a WaypointStamped or WaypointArray message. For the 
   first two options, the frames they are relative to must be given via 
   parameters (launch_frame, rviz_frame). For the last, the frame will be
   specified in the header. Any frame can be used, provided there is a 
   transformation from it to the odom_frame (which move_base uses) and the
   path_vis_frame (for path visualisation).
   
   The waypoints are stored in the frame that they are given, and converted
   as required. The exception to this is GPS points, which are stored as 
*/

using std::vector;
using std::cout;
using std::endl;
using std::string;
using geometry_msgs::PointStamped;
using geometry_msgs::PoseStamped;
using waypoint_follower_msgs::Waypoint;
using waypoint_follower_msgs::WaypointStamped;
using waypoint_follower_msgs::WaypointArray;

WaypointArray waypoint_array;          //list of waypoints to complete
int current_waypoint_index = -1;       //the index in waypoint_array
int id_counter = 0;                    //for generating ID's
string name_space;                     //prestring for frame ID's
string map_frame;                      //"map"
string odom_frame;                     //"name_space/odom"
string baselink_frame;                 //"name_space/base_link"
string launch_frame;				           //frame_id of launch file waypoints
string rviz_frame;                     //frame_id of waypoints added from RVIZ
string path_vis_frame;                 //desired frame_id of path visualisation
string move_base_name;                 //"move_base"
bool include_start_pose;               //include the start pose in path visualisation               

PoseStamped start_pose;

ros::Publisher  pub_waypoint_paths;
ros::Publisher  pub_completed_paths;
ros::Subscriber sub_add_waypoint;
ros::Subscriber sub_move_base_result;
ros::Subscriber sub_obs_waypoint;
ros::Subscriber sub_set_waypoints;
ros::Subscriber sub_upd_waypoints;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
MoveBaseClient* ac;

/**************************************************
 * Helper functions
 **************************************************/

// Convert GPS coordinates into values relative to the UTM frame.
PointStamped convertLatLongToUTM(double lati, double longi){
  cout << "Converting: ( " << lati << ", " << longi << ")" << endl;
  double utm_northing, utm_easting;
  string utm_zone;
  RobotLocalization::NavsatConversions::LLtoUTM(lati, longi, utm_northing, utm_easting, utm_zone);
  cout << cout.precision(10) << "Results: ( " << utm_easting << ", " << utm_northing << ")" << endl;
  cout << "zone: " << utm_zone << endl;
  geometry_msgs::PointStamped ps;
  ps.header.frame_id = "utm";
  ps.header.stamp = ros::Time(0);
  ps.point.x = utm_easting;
  ps.point.y = utm_northing;
  return ps;
}

// Populates wp with a pointer to the waypoint. Returns the index of this waypoint
// in waypoint_array.waypoints, or -1 if it does not exist.
//        ptr1 -> ptr2 -> waypoint 
// We are given ptr1 (**wp), then we follow it one step (*wp) and alter it be
// the location of the waypoint (&waypoint). 
int getWaypointByID( int id, WaypointStamped **ws ){
  for(vector<WaypointStamped>::iterator it = waypoint_array.waypoints.begin(); it != waypoint_array.waypoints.end(); ++it) {
    if( id == it->waypoint.id ){
      *ws = &(*it);         //*it = waypoint
      int index = it - waypoint_array.waypoints.begin(); 
      return index;
    }
  }
  return -1;
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
    tf::TransformListener listener;
    cout << "Time is: " << ros::Time::now() << endl;
    cout << "Looking up transform from " << frame_in << " to " << frame_out << endl;
    geometry_msgs::PoseStamped old_pose, new_pose;
    ros::Time now            = ros::Time(0);
    old_pose.pose            = p;
    old_pose.header.frame_id = frame_in;
    old_pose.header.stamp    = now;
    
    listener.waitForTransform(frame_in, frame_out, now, ros::Duration(20) );
    listener.transformPose(frame_out, old_pose, new_pose);
    
    p = new_pose.pose;
    
    return true;
  } catch (tf2::TransformException e){
    cout << "Failed to lookup transform from " << frame_in << " to " << frame_out << endl;
    return false;
  }
}

// Takes an observed waypoint and updates the waypoint array. Returns whether
// move_base should be notified that the current goal has changed.
bool updateWaypoints( WaypointStamped ws, bool add_if_new ){
  cout << "Updating waypoints!" << endl;
  bool no_goal_atm = current_waypoint_index == -1;
  int type = ws.waypoint.type;
  if( type < 4 ){                          //no ID
    cout << "waypoint had no ID" << endl;
    return false;
  }
  // Get the stored waypoint or add one.
  WaypointStamped* w;                             //stored waypoint
  int index = getWaypointByID( ws.waypoint.id, &w );
  if( index == -1 ){
    if( add_if_new ){
      cout << "adding waypoint" << endl;
      waypoint_array.waypoints.push_back(ws);
      if( no_goal_atm ){
        geometry_msgs::PoseStamped p;
        p.header.frame_id    = rviz_frame;
        p.header.stamp       = ros::Time::now();
        p.pose.orientation.w = 1;
        convertPoseFrame( p.pose, ws.header.frame_id, path_vis_frame );
        start_pose = p;
      }
      return no_goal_atm;
    } else {
      return false;
    }
  }
  // Update components based on type.
  bool notify = false;
  if( type >= 4 ) {
    type -= 4;
  }
  if( type >= 2 ) {
    if( index == current_waypoint_index && w->waypoint.completed != ws.waypoint.completed ){
      notify = true;
    }
    w->waypoint.completed = ws.waypoint.completed;
    type -= 2;
  }
  if( type > 0 ){
    if( index == current_waypoint_index && (
        w->waypoint.pose.position.x != ws.waypoint.pose.position.x ||
        w->waypoint.pose.position.y != ws.waypoint.pose.position.y ||
        w->waypoint.pose.position.z != ws.waypoint.pose.position.z ||
        w->waypoint.pose.orientation.x != ws.waypoint.pose.orientation.x ||
        w->waypoint.pose.orientation.y != ws.waypoint.pose.orientation.y ||
        w->waypoint.pose.orientation.z != ws.waypoint.pose.orientation.z ||
        w->waypoint.pose.orientation.w != ws.waypoint.pose.orientation.w )){
      notify = true;
    }
    // Convert to the desired frame.
    /*if( ws.header.frame_id != odom_frame ){
      cout << "transforming from " << ws.header.frame_id << " to " << odom_frame << endl;
      if( convertPoseFrame( ws.waypoint.pose, ws.header.frame_id, odom_frame ) ){
        w->waypoint.pose = ws.waypoint.pose;
      }
    } else {*/
      w->waypoint.pose = ws.waypoint.pose;
    //}
  }
  return notify;
}

/**************************************************
 * Publisher functions
 **************************************************/
 
// Publishes completed tasks for visualisation in RVIZ. Uses the path_vis_frame. 
// The first point in the path can be the robots starting point.
void publishCompletedPaths(){
  cout << "Publishing completed paths!" << endl;
  nav_msgs::Path p;
  p.header.seq      = 0;
  p.header.stamp    = ros::Time::now();
  p.header.frame_id = rviz_frame;
  vector<geometry_msgs::PoseStamped> vec;
  geometry_msgs::PoseStamped ps;
  // Add robot start position.
  if( include_start_pose ){
    vec.push_back( start_pose );
  }
  // Add waypoints.
  int end = current_waypoint_index;
  if( end < 0 ) end = waypoint_array.waypoints.size();
  for( int i=0; i<end; i++ ){
    WaypointStamped ws = waypoint_array.waypoints.at(i);
    ps.header = ws.header;
    ps.pose   = ws.waypoint.pose;
    if( ws.header.frame_id != rviz_frame ){
      convertPoseFrame( ps.pose, ws.header.frame_id, path_vis_frame );
    }
    ps.header.frame_id = path_vis_frame;
    vec.push_back( ps );
  }
  p.poses = vec;
  pub_completed_paths.publish( p );
}

// Publishes a path message for visualisation in RVIZ. Uses the rviz_frame. 
// The first point in the path can be the robots base_link.
void publishWaypointPaths(){
  cout << "Publishing waypoint paths!" << endl;
  nav_msgs::Path p;
  p.header.seq      = 0;
  p.header.stamp    = ros::Time::now();
  p.header.frame_id = rviz_frame;
  vector<geometry_msgs::PoseStamped> vec;
  geometry_msgs::PoseStamped ps;
  // Add robot start position.
  if( include_start_pose ){
    geometry_msgs::Pose pose;
    pose.orientation.w = 1;
    convertPoseFrame( pose, baselink_frame, path_vis_frame );
    ps.header.stamp = ros::Time::now();
    ps.header.frame_id = path_vis_frame;
    ps.pose = pose;
    vec.push_back( ps );
  }
  // Add waypoints.
  for( int i=current_waypoint_index; i<waypoint_array.waypoints.size(); i++ ){
    if( i < 0 ) continue;
    WaypointStamped ws = waypoint_array.waypoints.at(i);
    ps.header = ws.header;
    ps.pose   = ws.waypoint.pose;
    if( ws.header.frame_id != rviz_frame ){
      convertPoseFrame( ps.pose, ws.header.frame_id, path_vis_frame );
    }
    ps.header.frame_id = path_vis_frame;
    vec.push_back( ps );
  }
  p.poses = vec;
  pub_waypoint_paths.publish( p );
  publishCompletedPaths();
}

// Send a goal to move_base.
void sendGoalToMoveBase(WaypointStamped* ws){
  geometry_msgs::Pose p = ws->waypoint.pose;
  convertPoseFrame( p, ws->header.frame_id, odom_frame );
  p.position.z = 0;
  //p.orientation.w = 1;
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = odom_frame;
  goal.target_pose.header.stamp    = ros::Time::now();
  goal.target_pose.pose            = p;
  ac->sendGoal(goal);
  cout << "sending to move base!: " << endl;
  cout << "Waypoint details" << endl;
  cout << " - frame: " << ws->header.frame_id << endl;
  cout << " - xyz: (" << ws->waypoint.pose.position.x << " , " << ws->waypoint.pose.position.y << " , " << ws->waypoint.pose.position.z << ")" << endl;
  cout << "Converted to" << endl;
  cout << " - frame: " << odom_frame << endl;
  cout << " - xyz: (" << p.position.x << " , " << p.position.y << " , " << p.position.z << ")" << endl; 
  cout << " - q: (" << p.orientation.x << " , " << p.orientation.y << " , " << p.orientation.z << " , " << p.orientation.w << ")" << endl;

}
 
// Picks the first waypoint that is not completed, and sends that to move base.
void startWaypointFollowing() {
  cout << "Starting waypoint following" << endl;
  current_waypoint_index = -1;
  for( int i=0; i<waypoint_array.waypoints.size(); i++ ){
    Waypoint w = waypoint_array.waypoints[i].waypoint;
    bool completed = w.completed;
    if( !completed ){
      current_waypoint_index = i;
      break;
    }
  }
  cout << "current_waypoint_index: " << current_waypoint_index << endl;
  if( current_waypoint_index > -1 ){
    cout << "Starting waypoint following" << endl;
    sendGoalToMoveBase( &(waypoint_array.waypoints.at(current_waypoint_index)) );
  } else {
    // TO DO: CANCEL GOAL
  }
}

 /**************************************************
 * Subscriber functions
 **************************************************/
// Designed for input from RVIZ.
void callbackAddWaypoint(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  cout << "Received a waypoint!" << endl;
  WaypointStamped ws;
  ws.header.stamp    = ros::Time::now();
  ws.header.frame_id = rviz_frame;
  ws.waypoint.type = 5;
  ws.waypoint.id   = id_counter++;
  ws.waypoint.pose = msg->pose;
  if( updateWaypoints(ws, true) ){
    startWaypointFollowing();
  } 
  publishWaypointPaths();
}

// Clears the current array and sets it to this.
void callbackSetWaypoints(const WaypointArray::ConstPtr& msg) {
  cout << "Setting waypoints!" << endl;
  waypoint_array.waypoints.clear();
  WaypointArray wp = *msg;
  for( int i=0; i<wp.waypoints.size(); i++ ){
    WaypointStamped ws_to_add = wp.waypoints[i];
    updateWaypoints(ws_to_add, true);
  }
  startWaypointFollowing();
  publishWaypointPaths();
}

// Does not clear the current array. Waypoints that are already in the array 
// will be updated. Waypoints not in the array will be added to the end.
void callbackUpdateWaypoints(const WaypointArray::ConstPtr& msg) {
  cout << "Updating waypoints!" << endl;
  bool notify = false;
  WaypointArray wp = *msg;
  for( int i=0; i<wp.waypoints.size(); i++ ){
    WaypointStamped ws_to_add = wp.waypoints[i];
    if( updateWaypoints(ws_to_add, true) ){
      notify = true;
    }
  }
  if( notify ){
    startWaypointFollowing();
  }
  publishWaypointPaths();
}

// Does not clear the current array. Waypoints that are already in the array 
// will be updated. Waypoints not in the array will NOT be added.
void callbackObservedWaypoints(const WaypointArray::ConstPtr& msg) {
  cout << "Observing waypoints!" << endl;
  bool notify = false;
  WaypointArray wp = *msg;
  for( int i=0; i<wp.waypoints.size(); i++ ){
    WaypointStamped ws_to_add = wp.waypoints[i];
    if( updateWaypoints(ws_to_add, false) ){
      notify = true;
    }
  }
  if( notify ){
    startWaypointFollowing();
  }
  publishWaypointPaths();
}

// Receive the status of move_base. Called when move_base will no longer pursue
// the waypoint. 
// TO DO: ONLY MOVE TO THE NEXT WAYPOINT IF FEEDBACK TELLS US THIS IS COMPLETE
/*void callbackMoveBaseResult(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg){
  cout << "Result received" << endl;
  int status = msg->status.status;
  if( status == msg->status.SUCCEEDED ){
    cout << "Waypoint achieved!!!" << endl;
    waypoint_array.waypoints[current_waypoint_index].waypoint.completed = true;
    startWaypointFollowing();
    publishWaypointPaths();
  }
} */

// Called periodically, checks if move_base has completed its waypoint.
void checkMoveBase(){
  if( current_waypoint_index <= -1 ) return;
  /*cout << "Checking move base" << endl;
  if( ac->getState() == actionlib::SimpleClientGoalState::PENDING ){
    cout << "Pending" << endl;
  }
  if( ac->getState() == actionlib::SimpleClientGoalState::ACTIVE ){
    cout << "Active" << endl;
  }
  if( ac->getState() == actionlib::SimpleClientGoalState::RECALLED ){
    cout << "Recalled" << endl;
  }
  if( ac->getState() == actionlib::SimpleClientGoalState::REJECTED ){
    cout << "Rejected" << endl;
  }
  if( ac->getState() == actionlib::SimpleClientGoalState::PREEMPTED ){
    cout << "Preempted" << endl;
  }
  if( ac->getState() == actionlib::SimpleClientGoalState::ABORTED ){
    cout << "Aborted" << endl;
  }
  if( ac->getState() == actionlib::SimpleClientGoalState::LOST ){
    cout << "Lost" << endl;
  }*/
  if( ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED ){
    cout << "Waypoint achieved!!!" << endl;
    waypoint_array.waypoints[current_waypoint_index].waypoint.completed = true;
    startWaypointFollowing();
    publishWaypointPaths();
  }
}

/**************************************************
 * Initialising functions
 **************************************************/
 
void loadSubs(ros::NodeHandle n){
  sub_add_waypoint     = n.subscribe("add_waypoint",      100, callbackAddWaypoint);   
  sub_set_waypoints    = n.subscribe("set_waypoints",     100, callbackSetWaypoints);
  sub_upd_waypoints    = n.subscribe("update_waypoints",  100, callbackUpdateWaypoints);                                       
  sub_obs_waypoint     = n.subscribe("observed_waypoints",100, callbackObservedWaypoints);  
  //sub_move_base_result = n.subscribe("move_base/result",  100, callbackMoveBaseResult);  
}

void loadPubs(ros::NodeHandle n){
  pub_waypoint_paths  = n.advertise<nav_msgs::Path>("waypoint_paths", 100);
  pub_completed_paths = n.advertise<nav_msgs::Path>("completed_paths", 100);
}

// Loads parameters from the launch file. Uses default values if any values are
// missing.
void loadParams(ros::NodeHandle n_priv){
  vector<double> waypoint_values;    //position (x,y,z) and orientation (x,y,z,w)
  vector<double> gps_values;         //position (lat, long)
  
  // Set default parameters.
  double default_waypoint_values[] = {};
  double default_gps_values[]      = {};
  string default_baselink_frame  = "base_link";
  string default_map_frame       = "map";
  string default_odom_frame      = "odom";
  string default_move_base_name  = "move_base";
  bool default_include_start_pose = false;
  
  n_priv.param("baselink_frame",     baselink_frame,     default_baselink_frame);
  n_priv.param("map_frame",          map_frame,          default_map_frame);
  n_priv.param("odom_frame",         odom_frame,         default_odom_frame);
  n_priv.param("launch_frame",       launch_frame,       odom_frame);
  n_priv.param("rviz_frame",         rviz_frame,         odom_frame);
  n_priv.param("path_vis_frame",     path_vis_frame,     odom_frame);
  n_priv.param("include_start_pose", include_start_pose, default_include_start_pose);
  n_priv.param("move_base_name",     move_base_name,     default_move_base_name);
  
  // Check parameter server to override defaults.
  XmlRpc::XmlRpcValue v;
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
  if( waypoint_values.size() % 7 != 0 ){
    cout << "INCORRECT NUMBER OF WAYPOINT VALUES" << endl;
    return;
  }
  for( int i=0; i<waypoint_values.size(); i+=7 ) {
    WaypointStamped ws;
    ws.header.seq = 0;
    ws.header.stamp = ros::Time::now();
    ws.header.frame_id = launch_frame;
    ws.waypoint.type = 7;
    ws.waypoint.id = id_counter++;
    ws.waypoint.completed = false;
    ws.waypoint.pose.position.x = waypoint_values[i];
    ws.waypoint.pose.position.y = waypoint_values[i+1];
    ws.waypoint.pose.position.z = waypoint_values[i+2];
    ws.waypoint.pose.orientation.x = waypoint_values[i+3];
    ws.waypoint.pose.orientation.y = waypoint_values[i+4];
    ws.waypoint.pose.orientation.z = waypoint_values[i+5];
    ws.waypoint.pose.orientation.w = waypoint_values[i+6];
    waypoint_array.waypoints.push_back(ws);
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
  cout << "NUMBER OF GPS WAYPOINTS: " << gps_values.size()/2 << endl;
  for( int i=0; i<gps_values.size(); i+=2 ) {
    cout << "FIRST GPS PAIR LOADED" << endl;
    WaypointStamped ws;
    ws.waypoint.type = 7;
    ws.waypoint.id = id_counter++;
    ws.waypoint.completed = false;
    double lati = gps_values[i];
    double longi = gps_values[i+1];
    PointStamped ps = convertLatLongToUTM(lati, longi);
    convertUTMtoMap( ps );
    
    ws.header = ps.header;
    ws.waypoint.pose.position.x = ps.point.x;
    ws.waypoint.pose.position.y = ps.point.y;
    ws.waypoint.pose.position.z = ps.point.z;
    ws.waypoint.pose.orientation.x = 0;
    ws.waypoint.pose.orientation.y = 0;
    ws.waypoint.pose.orientation.z = 0;
    ws.waypoint.pose.orientation.w = 1;
    waypoint_array.waypoints.push_back(ws);
  }
}

/**************************************************
 * Main
 **************************************************/
int main(int argc, char** argv){
  ros::init(argc, argv, "waypoint_follower");
  ros::NodeHandle n;
  ros::NodeHandle n_priv("~");
  loadSubs(n);
  loadPubs(n);
  loadParams(n_priv);
  ac = new MoveBaseClient(move_base_name, true);
   
  if( waypoint_array.waypoints.size() > 0 ) {
    while(!ac->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
    }
    startWaypointFollowing();
    publishWaypointPaths();
  }
  
  ros::Rate loop_rate(1);
  while( ros::ok() ){
    checkMoveBase();
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  delete ac;
  return 0;
};


