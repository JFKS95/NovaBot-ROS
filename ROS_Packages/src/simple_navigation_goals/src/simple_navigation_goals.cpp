// This script places a waypoint on the map using the 2D navigation goal package

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <string>
#include <fstream>
#include <sstream>

using namespace std;


// A python script will display a top down view of a map, the user will click an area on the map which will
// generate a set of x, y coordinates and write them to a text file. This scriot will read the coordinates
// from that file and write them to the 2D Nav Goal package
fstream myfile;
float xFloat = 0;     // initalising x, y coordinates 
float yFloat = 0;
string xCoord = "x";
string yCoord = "y";

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  // read from the file, first line contains the x coordinate, second line the y coordinate
  myfile.open("//home//team-g//catkin_ws//src//simple_navigation_goals//src//TargetZone.txt");
  
  for(int line = 0; line < 2; line++)
  {
    if(line == 0)
    {
      getline(myfile, xCoord);  
    }
    if(line == 1)
    {
      getline(myfile, yCoord);
    }
  }
  
  // convert the string to a float
  istringstream (xCoord) >> xFloat; 
  istringstream (yCoord) >> yFloat;

  xFloat = xFloat * 0.0065;
  yFloat = yFloat * 0.0065;


  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = xFloat;
  goal.target_pose.pose.position.y = yFloat;
  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    //ROS_INFO("Hooray, the base moved 1 meter forward");
    ROS_INFO_STREAM(xFloat);
  else
    ROS_INFO("The base failed to move forward 1 meter for some reason");

  return 0;
}
