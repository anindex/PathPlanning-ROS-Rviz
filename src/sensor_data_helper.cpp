#include "Grid.h"


Grid scenario;

void set_map(const nav_msgs::OccupancyGrid::ConstPtr& msg){
  scenario.grid.header.frame_id = msg->header.frame_id;
  scenario.grid.header.stamp = msg->header.stamp;

  scenario.grid.info.resolution = msg->info.resolution;
  scenario.grid.info.width = msg->info.width;
  scenario.grid.info.height = msg->info.height;
  scenario.grid.info.origin.orientation.w = msg->info.origin.orientation.w;
  scenario.grid.data.resize(scenario.grid.info.width * scenario.grid.info.height);
  for(int j = 0; j <  scenario.grid.info.height; j++){
    for(int i = 0; i <  scenario.grid.info.width; i++){
      scenario.grid.data[scenario.grid.info.width * j + i] = msg->data[scenario.grid.info.width * j + i];
    }
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "sensor_data_helper");
  ros::NodeHandle n;
  n.getParam("sensor_range", scenario.sensor_range);
  ros::Subscriber sub = n.subscribe("real_map", 1, set_map);
  
  ros::ServiceServer ss = n.advertiseService("sensor_data", &Grid::detect, &scenario);
  ros::spin();
}
