#!/usr/bin/env python

import rospy
from dlite_simulator.msg import Cell
from dlite_simulator.srv import SensorData


def convert_data(data):
    walls = [(item.x, item.y) for item in data.black]
    space = [(item.x, item.y) for item in data.white]
    return walls, space

if __name__ == "__main__":
    p = Cell()
    p.x = 20
    p.y = 20

    rospy.wait_for_service("sensor_data")
    get_data = rospy.ServiceProxy("sensor_data", SensorData)
    data = get_data(p)
    walls, space = convert_data(data)
    print walls
    print space
