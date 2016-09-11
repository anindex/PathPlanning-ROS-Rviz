#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PointStamped, PoseStamped # for robot position
from nav_msgs.msg import OccupancyGrid, Path # for path and known map display
from time import sleep
from heapq import heappush, heappop, heapify
from dlite_simulator.msg import Cell
from dlite_simulator.srv import SensorData
import math

class PriorityDict(dict):
    def __init__(self, *args, **kwargs):
        super(PriorityDict, self).__init__(self, *args, **kwargs)
        self._rebuild_heap()

    def _rebuild_heap(self):
        self._heap = [(v, k) for k, v in self.iteritems()]
        heapify(self._heap)

    def pop(self):
        v, k = heappop(self._heap)
        while k not in self or self[k] != v:
            v, k = heappop(self._heap)
        del self[k]
        return k, v

    def top(self):
        heap  = self._heap
        v, k = heap[0]
        while k not in self or self[k] != v:
            heappop(heap)
            v, k = heap[0]
        return k, v

    def __setitem__(self, k, v):
        super(PriorityDict, self).__setitem__(k, v)

        if len(self._heap) < 2 * len(self):
            heappush(self._heap, (v, k))
        else:
            self._rebuild_heap()

class Robot:
    def __init__(self, startP, w, h):
        self.pos = startP
        self.knownWorld = []

        self.ix = w
        self.iy = h

    def passable(self, id):
        return id not in self.knownWorld

    def in_bounds(self, id):
        return 0 <= id[0] and id[0] < self.ix and 0 <= id[1] and id[1] < self.iy

    def cost(self, currP, nextP):
        if currP in self.knownWorld or nextP in self.knownWorld:
            return float("inf")
        val = 0
        if abs(nextP[1] - currP[1]) + abs(nextP[0] - currP[0]) == 1:
            val += 1
        elif abs(nextP[1] - currP[1]) + abs(nextP[0] - currP[0]) == 2:
            val += 1.414
        return val

    def detect_neighbor(self, id):
        x, y = id
        result = [(x + i, y + j) for i in range(-1, 2) for j in range(-1, 2)]
        result.remove(id)
        result = filter(self.in_bounds, result)
        result = filter(self.passable, result)
        if (x+y) % 2 == 0: result.reverse()
        return result

    def update_cell(self, id):
        if id in self.knownWorld:
            self.knownWorld.remove(id)
        else:
            self.knownWorld.append(id)

    def detect_changes(self, sensorData):
        toHigh = filter(lambda item: item not in self.knownWorld, sensorData[0])
        toLow = filter(lambda item: item in self.knownWorld, sensorData[1])
        return toHigh, toLow


def calculate_key(node, data, robotPos, km):
    return min(data[node][0], data[node][1]) + heuristic_distance(robotPos, node) + km

def update_vertex(node, robot, goal, frontier, data, km):
    if node != goal: data[node][1] = get_lowest_cost_node(robot, node, data)[1]
    if node in frontier: del frontier[node]
    if  data[node][0] != data[node][1]: frontier[node] = calculate_key(node, data, robot.pos, km)

def get_lowest_cost_node(robot, node, data):
    neighbors = {k: (data[k][0]+robot.cost(node, k)) for k in robot.detect_neighbor(node)}
    result = min(neighbors, key=neighbors.get)
    return result, neighbors[result]

def get_lowest_rhs_node(robot, node, data):
    neighbors = {k: (data[k][1]+robot.cost(node, k)) for k in robot.detect_neighbor(node)}
    result = min(neighbors, key=neighbors.get)
    return result, neighbors[result]

def compute_shortest_path(robot, goal, data, frontier, km):
    while frontier and (frontier.top()[1] < (calculate_key(robot.pos, data, robot.pos, km) + 1) or data[robot.pos][1] != data[robot.pos][0]):
        currNode, kold = frontier.pop()
        knew = calculate_key(currNode, data, robot.pos, km)
        if kold < knew:
            frontier[currNode] = knew
        elif data[currNode][0] > data[currNode][1]:
            data[currNode][0] = data[currNode][1]
            for pred in robot.detect_neighbor(currNode):
                update_vertex(pred, robot, goal, frontier, data, km)
        else:
            data[currNode][0] = float("inf")
            for node in robot.detect_neighbor(currNode) + [currNode, ]:
                update_vertex(node, robot, goal, frontier, data, km)

###########################################################
#                  Utility                                #
###########################################################

def modify_path(path, newPath, res):
    del path.poses[:]
    for pos in newPath:
        newPos = PoseStamped()
        newPos.pose.position.x = pos[0] * res
        newPos.pose.position.y = pos[1] * res
        newPos.pose.orientation.w = 1.0
        path.poses.append(newPos)

def heuristic_distance(startP, goal):
    return math.sqrt((startP[0] - goal[0])**2 + (startP[1] - goal[1])**2)

def heuristic_manhattan(startP, goal):
    return abs(startP[0] - goal[0]) + abs(startP[1] - goal[1])

def reconstruct_path_gradient(robot, data, goal):
    current = robot.pos
    path = [current]
    if data[current][1] == float("inf"):
        return None
    while current != goal:
        current = get_lowest_cost_node(robot, current, data)[0]
        path.append(current)
    return path

def convert_data(data):
    walls = [(item.x, item.y) for item in data.black]
    space = [(item.x, item.y) for item in data.white]
    return walls, space

if __name__ == "__main__":
    ########### Initialization ###############
    rospy.init_node("robot", anonymous=False)
    robot_pos_pub = rospy.Publisher("robot_pos", PointStamped, queue_size=2)
    known_map_pub = rospy.Publisher("known_map", OccupancyGrid, queue_size=10)
    path_pub = rospy.Publisher("path", Path, queue_size=5)
    rate = rospy.Rate(2)

    km = 0
    width = rospy.get_param("map_width")
    height =  rospy.get_param("map_height")

    data = {k: [float("inf"), float("inf")] for k in [(i, j) for i in range(width) for j in range(height)]}
    frontier = PriorityDict()

    startP = (rospy.get_param("~startX"), rospy.get_param("~startY")) # set start and goal here
    goal = (rospy.get_param("goalX"), rospy.get_param("goalY"))
    data[goal][1] = 0
    frontier[goal] = heuristic_distance(startP, goal)
    lastNode = startP

    ## For Robot
    robot = Robot(startP, width, height)

    ## For OccupancyGrid
    known_world = OccupancyGrid()
    known_world.header.frame_id = "/known_map"

    known_world.info.width = width
    known_world.info.height = height
    known_world.info.resolution = rospy.get_param("resolution")
    known_world.info.origin.orientation.w = 1.0

    for i in range(width*height):
        known_world.data.append(0)

    known_map_pub.publish(known_world)

    ## For Robot Position
    pos = PointStamped()
    pos.header.frame_id ="/map"
    pos.point.x = robot.pos[0] * known_world.info.resolution
    pos.point.y = robot.pos[1] * known_world.info.resolution
    robot_pos_pub.publish(pos)

    ## For Path
    curr_path = Path()
    curr_path.header.frame_id = "/map"

    ## get sensor data
    rospy.wait_for_service("sensor_data")
    get_data = rospy.ServiceProxy("sensor_data", SensorData)
    raw_data = get_data(Cell(startP[0], startP[1]))
    sensorData = convert_data(raw_data)
    changedNode = robot.detect_changes(sensorData)
    for node in changedNode[0] + changedNode[1]:
        robot.update_cell(node)
    ########## Main #########################

    compute_shortest_path(robot, goal, data, frontier, km)
    modify_path(curr_path, reconstruct_path_gradient(robot, data, goal), known_world.info.resolution)
    path_pub.publish(curr_path)
    while robot.pos != goal:
        # set move
        robot.pos = get_lowest_cost_node(robot, robot.pos, data)[0]
        pos.point.x = robot.pos[0] * known_world.info.resolution
        pos.point.y = robot.pos[1] * known_world.info.resolution
        robot_pos_pub.publish(pos)

        #detect change
        raw_data = get_data(Cell(robot.pos[0], robot.pos[1]))
        sensorData = convert_data(raw_data)
        changedNode = robot.detect_changes(sensorData)
        if changedNode[0] or changedNode[1]:
            km += heuristic_distance(lastNode, robot.pos)
            lastNode = robot.pos
            for node in changedNode[0] + changedNode[1]:
                robot.update_cell(node)                # node -> v, next -> u
                known_world.data[node[1]*width + node[0]] = 100 if node in changedNode[0] else 0
                for next in robot.detect_neighbor(node):
                    update_vertex(next, robot, goal, frontier, data, km)
                update_vertex(node, robot, goal, frontier, data, km)
            compute_shortest_path(robot, goal, data, frontier, km)
            modify_path(curr_path, reconstruct_path_gradient(robot, data, goal), known_world.info.resolution)
            path_pub.publish(curr_path)
            known_map_pub.publish(known_world)
        rate.sleep()
    rospy.loginfo("Goal Reach!!!")
    rospy.spin()
