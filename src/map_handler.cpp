#include "Grid.h"
#include <geometry_msgs/PointStamped.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "map_handler");
  ros::NodeHandle n;
  float res;
  int w, h, sensor_range;
  n.getParam("resolution", res);
  n.getParam("map_width", w);
  n.getParam("map_height", h);
  n.getParam("sensor_range", sensor_range);
  Grid scenario = map1(res, w, h, sensor_range);

  ros::Publisher pub = n.advertise<nav_msgs::OccupancyGrid>("/real_map", 10);
  ros::Publisher goal_pub = n.advertise<geometry_msgs::PointStamped>("/goal_robot", 10);

  geometry_msgs::PointStamped goal;
  goal.header.frame_id = "/map";
  n.getParam("goalX", goal.point.x);
  goal.point.x *= res;
  n.getParam("goalY", goal.point.y);
  goal.point.y *= res;
  ros::Rate r(10);

  while(ros::ok()){
    scenario.upload(&pub);
    goal_pub.publish(goal);
    r.sleep();
  }
}
