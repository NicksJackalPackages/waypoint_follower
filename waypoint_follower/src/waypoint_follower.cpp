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

/* Written by Nick Sullivan, The University of Adelaide. 
   This file takes a series of waypoints and periodically sends them to 
   move_base. The waypoints can come from a launch file, or dynamically added.
   Also supports feedback i.e. if the waypoint is observed in a new position,
   the waypoint is updated and re-sent to move_base. It is an option to not
   move to the next waypoint until feedback has seen that the waypoint is 
   complete.
   The waypoints are in the 'odom' frame, so they are relative to the robots
   start position.
*/

using std::vector;
using std::cout;
using std::endl;
using std::string;
using waypoint_follower_msgs::Waypoint;
using waypoint_follower_msgs::WaypointStamped;
using waypoint_follower_msgs::WaypointArray;

WaypointArray waypoint_array;          //list of waypoints to complete
int current_waypoint_index = -1;       //the index in waypoint_array
int id_counter = 0;                    //for generating ID's
string name_space;                     //prestring for frame ID's
string frame_id;                       //"name_space/odom"
string waypoints_frame;				   //frame_id of launch file waypoints

//ros::Publisher  pub_waypoint_poses;
ros::Publisher  pub_waypoint_paths;
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

// Given a pose belonging to frame_in, changes it to if it were from frame_out.
// Returns whether it was successful.
bool convertPoseFrame( geometry_msgs::Pose &p, string frame_in, string frame_out ){
  try {
    geometry_msgs::PoseStamped old_pose, new_pose;
    old_pose.pose            = p;
    old_pose.header.frame_id = frame_in;
    old_pose.header.stamp    = ros::Time(0);
    tf::TransformListener listener;
    listener.waitForTransform(frame_in, frame_out, ros::Time(0), ros::Duration(0.5) );
    //tf::StampedTransform transform;
    //listener.lookupTransform(frame_in, frame_out, ros::Time(0), transform);
    //cout << "looked up transform" << endl;
    listener.transformPose(frame_out, old_pose, new_pose);
    //cout << "old x: " << old_pose.pose.position.x << endl;
    //cout << "old y: " << old_pose.pose.position.y << endl;
    //cout << "old z: " << old_pose.pose.position.z << endl;
    //cout << "new x: " << new_pose.pose.position.x << endl;
    //cout << "new y: " << new_pose.pose.position.y << endl;
    //cout << "new z: " << new_pose.pose.position.z << endl;
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
  bool no_goal_atm = current_waypoint_index == -1;
  int type = ws.waypoint.type;
  if( type < 4 ){                          //no ID
    return false;
  }
  // Get the stored waypoint or add one.
  WaypointStamped* w;                             //stored waypoint
  int index = getWaypointByID( ws.waypoint.id, &w );
  if( index == -1 ){
    if( add_if_new ){
      /*if( ws.header.frame_id != frame_id ){
        convertPoseFrame( ws.waypoint.pose, ws.header.frame_id, frame_id );
        ws.header.frame_id = frame_id;
      }*/
      waypoint_array.waypoints.push_back(ws);
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
    /*if( ws.header.frame_id != frame_id ){
      cout << "transforming from " << ws.header.frame_id << " to " << frame_id << endl;
      if( convertPoseFrame( ws.waypoint.pose, ws.header.frame_id, frame_id ) ){
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
 
// Publishes a path message for visualisation in RVIZ. Uses the waypoints_frame
// because RVIZ ignores the individual frames.
void publishWaypointPaths(){
  nav_msgs::Path p;
  p.header.seq      = 0;
  p.header.stamp    = ros::Time::now();
  p.header.frame_id = waypoints_frame;
  vector<geometry_msgs::PoseStamped> vec;
  geometry_msgs::PoseStamped ps;
  for( int i=current_waypoint_index; i<waypoint_array.waypoints.size(); i++ ){
    if( i < 0 ) continue;
    WaypointStamped ws = waypoint_array.waypoints.at(i);
    ps.header = ws.header;
    ps.pose   = ws.waypoint.pose;
    vec.push_back( ps );
  }
  p.poses = vec;
  pub_waypoint_paths.publish( p );
}

// Publishes the poses of the waypoints in the waypoint array. This is for 
// visualising in RVIZ.
/*void publishWaypointPoses(){
  geometry_msgs::PoseArray pose_array;
  pose_array.header.seq      = 0;
  pose_array.header.stamp    = ros::Time::now();
  pose_array.header.frame_id = frame_id;
  vector<geometry_msgs::Pose> vec;
  for( int i=current_waypoint_index; i<waypoint_array.waypoints.size(); i++ ){
    if( i < 0 ) continue;
    geometry_msgs::Pose p = waypoint_array.waypoints.at(i).waypoint.pose;
    if( waypoint_array.waypoints.at(i).header.frame_id != frame_id ){
      convertPoseFrame( p, waypoint_array.waypoints.at(i).header.frame_id, frame_id);
    }
    vec.push_back( p );
  }
  pose_array.poses = vec;
  pub_waypoint_poses.publish(pose_array);
} */
 
// Send a goal to move_base.
void sendGoalToMoveBase(WaypointStamped* ws){
  //cout << "sending to move base!: " << frame_id << endl;
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = ws->header.frame_id;
  goal.target_pose.header.stamp    = ros::Time::now();
  goal.target_pose.pose            = ws->waypoint.pose;
  //cout << "frame: " << ws->header.frame_id << endl;
  //cout << "x: " << ws->waypoint.pose.position.x << endl;
  //cout << "y: " << ws->waypoint.pose.position.y << endl;
  ac->sendGoal(goal);
}
 
// Picks the first waypoint that is not completed, and sends that to move base.
void startWaypointFollowing() {
  current_waypoint_index = -1;
  for( int i=0; i<waypoint_array.waypoints.size(); i++ ){
    Waypoint w = waypoint_array.waypoints[i].waypoint;
    bool completed = w.completed;
    if( !completed ){
      current_waypoint_index = i;
      break;
    }
  }
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
// Designed for input from RVIZ. Assumes values in the odom frame.
void callbackAddWaypoint(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  WaypointStamped ws;
  ws.header.stamp    = ros::Time::now();
  ws.header.frame_id = frame_id;
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
void callbackMoveBaseResult(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg){
  cout << "Result received" << endl;
  int status = msg->status.status;
  if( status == msg->status.SUCCEEDED ){
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
  sub_move_base_result = n.subscribe("move_base/result",  100, callbackMoveBaseResult);  
  sub_obs_waypoint     = n.subscribe("observed_waypoints",100, callbackObservedWaypoints);  
  sub_set_waypoints    = n.subscribe("set_waypoints",     100, callbackSetWaypoints);
  sub_upd_waypoints    = n.subscribe("update_waypoints",  100, callbackUpdateWaypoints);                                       
}

void loadPubs(ros::NodeHandle n){
  //pub_waypoint_poses = n.advertise<geometry_msgs::PoseArray>("waypoint_poses", 100);
  pub_waypoint_paths = n.advertise<nav_msgs::Path>("waypoint_paths", 100);
}

// Loads parameters from the launch file. Uses default values if any values are
// missing.
void loadParams(ros::NodeHandle n_priv){
  vector<double> waypoint_values;    //position (x,y,z) and orientation (x,y,z,w)
  
  // Set default parameters.
  double default_waypoint_values[] = {};
  string default_odom_frame      = "odom";
  
  n_priv.param("odom_frame",      frame_id, default_odom_frame);
  n_priv.param("waypoints_frame", waypoints_frame, frame_id);
  
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
    ws.header.frame_id = waypoints_frame;
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
  
}

/**************************************************
 * Main
 **************************************************/
int main(int argc, char** argv){
  ros::init(argc, argv, "waypoint_follower");
  ros::NodeHandle n;
  ros::NodeHandle n_priv("~");
  ac = new MoveBaseClient("move_base", true);
  loadSubs(n);
  loadPubs(n);
  loadParams(n_priv);
   
  if( waypoint_array.waypoints.size() > 0 ) {
    while(!ac->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
    }
    startWaypointFollowing();
    publishWaypointPaths();
  }
  cout << "Starting!" << endl;
  
  ros::spin();
  
  /*ros::Rate r(5);
  while( ros::ok() ){
    publishWaypointPaths();
    ros::spinOnce();
    r.sleep();
  }*/
  
  delete ac;
  return 0;
};


