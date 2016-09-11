#pragma once

#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <string>
#include <stdlib.h>
#include <time.h>
#include "dlite_simulator/SensorData.h"
#include <geometry_msgs/Point.h>
#include "dlite_simulator/Cell.h"
#include <iostream>
#define WALL 100

struct Point{
  int x;
  int y;
  Point(int a, int b) : x(a), y(b){ }
};

struct Grid{
  nav_msgs::OccupancyGrid grid;
  int sensor_range; // set range heare

  Grid(){}

  Grid(std::string frame_id, ros::Time t, float res, int w, int h, int sensor_range){
    grid.header.frame_id = frame_id;
    grid.header.stamp = t;

    grid.info.resolution = res;
    grid.info.width = w;
    grid.info.height = h;
    grid.info.origin.orientation.w = 1.0;
    grid.data.resize(grid.info.width * grid.info.height);

    this->sensor_range = sensor_range;
  }

  void add_rect(Point p1, Point p2){
    for(int i = p1.x; i < p2.x; i++){
      for(int j = p1.y; j < p2.y; j++){
        grid.data[grid.info.width * j + i] = WALL;
      }
    }
  }

  Point get_size(){
    return Point(grid.info.width, grid.info.height);
  }

  void set_point(Point p){
    grid.data[grid.info.width * p.y + p.x] = WALL;
  }

  void upload(ros::Publisher* pub){
    grid.info.map_load_time = ros::Time::now();
    pub->publish(grid);
  }

  void randomize_pixel(float scale){
    srand(time(NULL));
    int size_scale = scale*(grid.info.width*grid.info.height);
    int size_grid = grid.info.width*grid.info.height;
    for(int i = 0; i < size_scale; i++){
      int ind = rand() % size_grid;
      grid.data[ind] = 100;
    }
  }

  bool in_bounds(dlite_simulator::Cell& pt){
    return 0 <= pt.x && pt.x < grid.info.width && 0 <= pt.y && pt.y < grid.info.height;
  }

  bool detect(dlite_simulator::SensorData::Request &req, dlite_simulator::SensorData::Response &res){
    for (int i = -sensor_range; i <= sensor_range; i++){
      for(int j = -sensor_range; j <= sensor_range; j++){
        if (i == 0 && j == 0){
          continue;
        }
        else{
          dlite_simulator::Cell pt;
          pt.x = req.p.x + i;
          pt.y = req.p.y + j;
          if (in_bounds(pt)){
            if(grid.data[grid.info.width * pt.y + pt.x] == WALL){
              res.black.push_back(pt);
            }
            else{
              res.white.push_back(pt);
            }
          }
        }
      }
    }
    return true;
  }
};




Grid map1(float res, int w, int h, int sensor_range){
  Grid scenario("/map", ros::Time::now(), res, w, h, sensor_range);
  scenario.randomize_pixel(0.02);
  scenario.add_rect(Point(40, 20), Point(60, 30));
  scenario.add_rect(Point(10, 10), Point(20, 20));
  scenario.add_rect(Point(30,30), Point(70, 35));
  scenario.add_rect(Point(30,55), Point(70, 60));
  scenario.add_rect(Point(65,35), Point(70, 55));
  return scenario;
}
