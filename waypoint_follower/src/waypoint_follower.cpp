#include <ros/ros.h>
#include <ros/console.h>
#include <vector>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "waypoint_follower_msgs/Waypoint.h"
#include "waypoint_follower_msgs/WaypointArray.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>


/* Written by Nick Sullivan, The University of Adelaide. 
   This file takes a series of waypoints and periodically sends them to 
   move_base. The waypoints can come from a launch file, added one by one
   from RVIZ, or all at once by a topic. 
   Also supports feedback i.e. if the waypoint is observed in a new position,
   the waypoint is updated and re-sent to move_base. It is an option to not
   move to the next waypoint until feedback has seen that the waypoint is 
   complete.*/

using namespace std;
using namespace waypoint_follower_msgs;

WaypointArray waypoint_array;          //list of waypoints to complete
int current_waypoint_index = 0;        //the index in waypoint_array
int id_counter = 0;                    //for generating ID's
string name_space;                     //prestring for frame ID's
string frame_id;                       //"name_space/odom"

ros::Publisher  pub_waypoint_poses;
ros::Subscriber sub_add_waypoint;
ros::Subscriber sub_move_base_result;
ros::Subscriber sub_obs_waypoint;
ros::Subscriber sub_set_waypoints;

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
int getWaypointByID( int id, Waypoint **wp ){
  for(vector<Waypoint>::iterator it = waypoint_array.waypoints.begin(); it != waypoint_array.waypoints.end(); ++it) {
    if( id == it->id ){
      *wp = &(*it);         //*it = waypoint
      int index = it - waypoint_array.waypoints.begin(); 
      return index;
    }
  }
  return -1;
}

/**************************************************
 * Publisher functions
 **************************************************/
 
// Publishes the poses of the waypoints in the waypoint array. This is for 
// visualising in RVIZ.
void publishWaypointPoses(){
   geometry_msgs::PoseArray pose_array;
   pose_array.header.seq      = 0;
   pose_array.header.stamp    = ros::Time::now();
   pose_array.header.frame_id = frame_id;
   vector<geometry_msgs::Pose> vec;
   for( int i=current_waypoint_index; i<waypoint_array.waypoints.size(); i++ ){
       vec.push_back( waypoint_array.waypoints.at(i).pose );
   }
   pose_array.poses = vec;
   pub_waypoint_poses.publish(pose_array);
 } 
 
// Send a goal to move_base.
void sendGoalToMoveBase(Waypoint* waypoint){
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = frame_id;
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose = waypoint->pose;
  ac->sendGoal(goal);
}
 
 /**************************************************
 * Subscriber functions
 **************************************************/

// Clears the current array and sets it to this.
void callbackSetWaypoints(const WaypointArray::ConstPtr& msg) {
  waypoint_array = *msg;
  current_waypoint_index = 0;
  if( waypoint_array.waypoints.size() > 0 ) {
    sendGoalToMoveBase( &(waypoint_array.waypoints.at(0)) );
  } else {
    //TO DO, CANCEL GOAL
  }
}

// Designed for input from RVIZ.
void callbackAddWaypoint(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
  Waypoint waypoint;
  waypoint.type = 5;
  waypoint.id   = id_counter++;
  waypoint.pose = msg->pose.pose;
  waypoint_array.waypoints.push_back(waypoint);
  publishWaypointPoses();
  if( waypoint_array.waypoints.size() - current_waypoint_index == 1 ) {
    sendGoalToMoveBase( &(waypoint_array.waypoints.at(current_waypoint_index)) );
  }
  return;
}

// Receive the status of move_base. Called when move_base will no longer pursue
// the waypoint. 
// TO DO: ONLY MOVE TO THE NEXT WAYPOINT IF FEEDBACK TELLS US THIS IS COMPLETE
void callbackMoveBaseResult(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg){
  cout << "Result received" << endl;
  int status = msg->status.status;
  if( status == msg->status.SUCCEEDED ){
    cout << "Waypoint achieved!!!" << endl;
    current_waypoint_index++;
    if( current_waypoint_index < waypoint_array.waypoints.size() )
      sendGoalToMoveBase(&( waypoint_array.waypoints.at(current_waypoint_index) ) );
  }
} 

// Receive the state of observed waypoints.
void callbackObservedWaypoints(const WaypointArray::ConstPtr& msg) {
  vector<Waypoint> vec = msg->waypoints;
  for(vector<Waypoint>::iterator it = vec.begin(); it != vec.end(); ++it) {
       int type = it->type;
       // No shirt, no shoes, no ID, no service.
       if( type < 4 ){
         continue;
       }
       // Get the relevant waypoint that's been seen.
       int id = it->id;
       Waypoint* wp;
       int index = getWaypointByID( id, &wp );
       if ( index == -1 ){
        continue;
       }
       // Update pose.
       if( type == 5 || type == 7 ){
          // If the pose is within some tolerance, don't bother.
          // if TO DO
          // TO DO
          wp->pose = it->pose;
       }
       // Update completion status.
       if( type == 6 || type == 7 ){
          wp->completed = it->completed;
       }
       // Possibly resend the goal.
       if( index == current_waypoint_index ){
          sendGoalToMoveBase( wp );
       }
       publishWaypointPoses();
   }
}

/**************************************************
 * Initialising functions
 **************************************************/
 
void loadSubs(ros::NodeHandle n){
  sub_add_waypoint     = n.subscribe("/initialpose",     100, callbackAddWaypoint);   
  sub_move_base_result = n.subscribe("move_base/result", 100, callbackMoveBaseResult);  
  sub_obs_waypoint     = n.subscribe("observed_waypoint",100, callbackObservedWaypoints);  
  sub_set_waypoints    = n.subscribe("set_waypoints",    100, callbackSetWaypoints);                                    
}

void loadPubs(ros::NodeHandle n){
  pub_waypoint_poses = n.advertise<geometry_msgs::PoseArray>("waypoint_poses", 100);
}

// Loads parameters from the launch file. Uses default values if any values are
// missing.
void loadParams(ros::NodeHandle n_priv){
  vector<double> waypoint_values;    //position (x,y,z) and orientation (x,y,z,w)
  
  // Set default parameters.
  double default_waypoint_values[] = {};
  
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
    Waypoint w;
    w.type = 7;
    w.id = id_counter++;
    w.completed = false;
    w.pose.position.x = waypoint_values[i];
    w.pose.position.y = waypoint_values[i+1];
    w.pose.position.z = waypoint_values[i+2];
    w.pose.orientation.x = waypoint_values[i+3];
    w.pose.orientation.y = waypoint_values[i+4];
    w.pose.orientation.z = waypoint_values[i+5];
    w.pose.orientation.w = waypoint_values[i+6];
    waypoint_array.waypoints.push_back(w);
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
  name_space = n.getNamespace().substr(1, string::npos);
  std::ostringstream strs;
  strs << name_space;
  strs << "/odom";
  frame_id = strs.str();
   
  if( waypoint_array.waypoints.size() > 0 ) {
    while(!ac->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
    }
    sendGoalToMoveBase( &( waypoint_array.waypoints.at(0) ) );
  }
  cout << "Starting!" << endl;
  
  ros::Rate r(5);
  while( ros::ok() ){
    publishWaypointPoses();
    ros::spinOnce();
    r.sleep();
  }
  
  delete ac;
  return 0;
};
