#! /usr/bin/env python
# 
# # ENPM 661 - Planning for Autonomous Robots:
# Project 3 Phase 3 - A_star on Turtlebot3
# Shon Cortes, Bo-Shiang Wang

from logging import shutdown
import numpy as np
import matplotlib.pyplot as plt
import rospy
from geometry_msgs.msg import Twist, Point


width = 10
height = 10
r = .066 / (float(2)) # Wheel radius in meters
L = .160  # Wheel base in meters

# Initialize your ROS node
rospy.init_node("move_robot")
# Set up a publisher to the /cmd_vel topic
pub = rospy.Publisher("cmd_vel", Twist, queue_size=5)
# Declare a message of type Twist
velocity_msg = Twist()
rate = rospy.Rate(4)  # 4 Hz


# Class for storing node position, cost to come, parent index, and prev_orientation.
class Node:
    def __init__(self, x, y, cost, parent_index, prev_orientation, curr_orientation, UL, RL, UL_prev, RL_prev):
        self.x = x
        self.y = y
        self.cost = cost
        self.parent_index = parent_index
        self.prev_orientation = prev_orientation
        self.curr_orientation = curr_orientation
        self.UL = UL
        self.RL = RL
        self.UL_prev = UL_prev
        self.RL_prev = RL_prev


def move_check(child_node):  # Check if the move is allowed.

    # Check if out of puzzle boundary
    if child_node.x < 0 or child_node.y < 0 or child_node.x >= width or child_node.y >= \
            height:
        return False

    # Check if obstacle
    elif obstacles_chk(child_node):
        return False

    else:
        return True


# Check if position is in Robot Adjusted obstacle space.
# Obstacle space was expanded by a radius of 10 + 5 for clearance for a total of 15.
# Warning obstacles appear larger than they are.
def obstacles_chk(NODE):
    node = [NODE.x, NODE.y]

    clearance = L * 2

    # Square
    if .50 - clearance <= node[0] <= 2.00 + clearance and 4.25 - clearance <= node[1] <= 5.75 + clearance:
        return True

    # Rectangle
    elif 3.75 - clearance <= node[0] <= 6.25 + clearance and 4.25 - clearance <= node[1] <= 5.75 + clearance:
        return True

    elif 7.25 - clearance <= node[0] <= 8.75 + clearance and 2.00 - clearance <= node[1] <= 4.00 + clearance:
        return True

    # Circle
    elif (node[0] - 2.00) ** 2 + (node[1] - 2.00) ** 2 <= (1.00 + clearance) ** 2:
        return True

    elif (node[0] - 2.00) ** 2 + (node[1] - 8.00) ** 2 <= (1.00 + clearance) ** 2:
        return True

    else:
        return False


def begin():  # Ask for user input of start and goal pos. Start and goal much be positive integers
    while True:

        # Start position
        start_x = .5
        start_y = .5

        # Goal position
        goal_x = 9
        goal_y = 9

        # Start orientation
        start_theta = 0

        # Goal threshold
        step_size = .5

        prev_orientation = start_theta

        # Wheel RPM values
        RPM_left = 7  
        RPM_right = 10  

        # Initialize start and goal nodes from node class
        start_node = Node(start_x, start_y, 0, -1, prev_orientation, prev_orientation, 0, 0, 0, 0)
        goal_node = Node(goal_x, goal_y, 0, -1, 0, 0, RPM_left, RPM_right, 0, 0)

        # Check if obstacle
        if obstacles_chk(start_node):
            print("Start position is in an obstacle.")
        elif obstacles_chk(goal_node):
            print("Goal position is in an obstacle.")

        # Check if values are positive and within the map
        elif start_node.x < 0 or start_node.y < 0 or start_node.x > width or start_node.y > \
                height:
            print("Please enter positive integer values (0 <= x <= 400, 0 <= y <= 300).")
        elif goal_node.x < 0 or goal_node.y < 0 or goal_node.x > width or goal_node.y > \
                height:
            print("Please enter positive integer values (0 <= x <= 400, 0 <= y <= 300).")

        else:
            break

    return start_node, goal_node, step_size, RPM_left, RPM_right


def euclidean_dist(goal_node, node):  # Calculate cost to goal
    dist = np.sqrt((goal_node.x - node.x) ** 2 + (goal_node.y - node.y) ** 2)
    return dist


def action(Xi, Yi, theta_i, UL, UR):
    # Xi, Yi,theta_i: Input point's coordinates
    # Xs, Ys: Start point coordinates for plot function
    # Xn, Yn, theta_n: End point coordinates
    # r: wheel radius
    # L: distance between two wheels
    global r, L
    t = 0
    dt = 0.1
    Xn = Xi
    Yn = Yi
    theta_n = theta_i # New orientation
    cost = 0 
    scale = 2.5
    while t < 1:
        t = t + dt
        Xs = Xn
        Ys = Yn
        Xn += (0.5 * r * (UL + UR) * np.cos(theta_n) * dt) * scale
        Yn += (0.5 * r * (UL + UR) * np.sin(theta_n) * dt) * scale
        theta_n += ((r / L) * (UR - UL) * dt) 
        cost += np.sqrt((0.5 * r * (UL + UR) * np.cos(theta_n) * dt)**2
                    + (0.5 * r * (UL + UR) * np.sin(theta_n) * dt)**2) # Cost to come


        node = Node(Xn, Yn, cost, 0, theta_n, theta_i, UL, UR, 0, 0)

        if move_check(node):  # Check if child is within the map or in an obstacle.
            pass
        else:  # If out of bounds or an obstacle, restart loop and choose new node.
            break

    theta_n = np.rad2deg(theta_n)

    return [Xn, Yn, cost, theta_n, theta_i, UL, UR]

def visualize_action(Xi, Yi, theta_i, UL, UR, color="blue"):
    # Xi, Yi,theta_i: Input point's coordinates
    # Xs, Ys: Start point coordinates for plot function
    # Xn, Yn, theta_n: End point coordinates
    # r: wheel radius
    # L: distance between two wheels
    global r, L
    t = 0
    dt = 0.1
    Xn = Xi
    Yn = Yi
    theta_n = np.deg2rad(theta_i) # New orientation
    cost = 0 
    scale = 2.5
    while t < 1:
        t = t + dt
        Xs = Xn
        Ys = Yn
        Xn += (0.5 * r * (UL + UR) * np.cos(theta_n) * dt) * scale
        Yn += (0.5 * r * (UL + UR) * np.sin(theta_n) * dt) * scale
        theta_n += ((r / L) * (UR - UL) * dt)

        node = Node(Xn, Yn, cost, 0, theta_n, theta_i, UL, UR, 0, 0)

        if move_check(node):  # Check if child is within the map or in an obstacle.
            pass
        else:  # If out of bounds or an obstacle, restart loop and choose new node.
            continue

        plt.plot([Xs, Xn], [Ys, Yn], color)

def motion_model(prev_orientation, RPM_left, RPM_right, x, y):

    theta = np.deg2rad(prev_orientation)
    # Get the new x and y coordinates, new theta, and cost through action()
    scale = 1.1
    wheel_speed = [[RPM_left / scale, RPM_left],
                    [RPM_left, RPM_left / scale],
                    [RPM_left, RPM_left],
                    [RPM_right / scale, RPM_right],
                    [RPM_right, RPM_right / scale],
                    [RPM_right, RPM_right],
                    [RPM_left, RPM_right],
                    [RPM_right, RPM_left]]

    model = []
    for w in wheel_speed:
        model.append(action(x, y, theta, w[0], w[1]))
    return model


def a_star(start_node, goal_node, step_size, RPM_left, RPM_right):
    # Initialize dictionaries
    path, distance, queue, visited = dict(), dict(), dict(), dict()

    queue[(start_node.x, start_node.y)] = start_node  # Initialize queue with startnode for Dijkstra algorithm.
    distance[(start_node.x, start_node.y)] = 0  # Initialize distance traveled.

    threshold = 0.5

    # Dictionary for theta
    orientation_dict = {0: 0, 30: 1, 60: 2, 90: 3,
                        120: 4, 150: 5, 180: 6, 210: 7,
                        240: 8, 270: 9, 300: 10, 330: 11, 360: 0}

    # Create V matrix to store the information of the visited nodes.
    V = np.zeros((int(width / threshold) + 1, int(height / threshold) + 1, int(360 / 30)))

    while True:  # Start of A star Algorithm.
        # Find the node in queue with the minimum cost.
        cur_index = min(queue, key=lambda o: queue[o].cost + euclidean_dist(goal_node, queue[o])) # Assign node in queue with minimum cost to be the current node to be tested.
        cur = queue[cur_index]
        prev_orientation = cur.prev_orientation
        UL_prev = cur.UL
        RL_prev = cur.RL

        # If goal node is reached, Break the while loop.
        # Add a threshold(circle) for the goal node
        if (goal_node.x - cur.x) ** 2 + (goal_node.y - cur.y) ** 2 <= (1.5 * step_size) ** 2:

            goal_node.parent_index = cur.parent_index
            goal_node.cost = cur.cost
            goal_node.curr_orientation = cur.curr_orientation
            goal_node.UL = cur.UL
            goal_node.RL = cur.RL
            print('Goal Found')
            break

        del queue[cur_index]  # Remove the current node from the queue.
        visited[(cur.x, cur.y, cur.prev_orientation)] = cur  # Add current node to visited list.

        # Mark 1 for visited nodes in matrix V
        a = int(round(cur.x) / threshold)
        b = int(round(cur.y) / threshold)
        c = int(prev_orientation // 30)
        V[a][b][c] = 1

        # Initialize action set with prev_orientation, RPM_left, RPM_right, cur.x, cur.y
        motion = motion_model(prev_orientation, RPM_left, RPM_right, cur.x, cur.y)

        # Generate children of current node based on the action set.
        for i in range(len(motion)):
            next_x = round(motion[i][0], 3)
            next_y = round(motion[i][1], 3)
            child_orientation = round(motion[i][3])
            UL = motion[i][5]
            RL = motion[i][6]

            # Generate child node
            node = Node(next_x, next_y, cur.cost + motion[i][2], cur_index, child_orientation, prev_orientation, UL, RL, UL_prev, RL_prev)
            # Assign child node position
            node_index = (node.x, node.y)

            if move_check(node):  # Check if child is within the map or in an obstacle.
                pass
            else:  # If out of bounds or an obstacle, restart loop and choose new node.
                continue

            a = int(round(node.x))
            b = int(round(node.y))

            if node.prev_orientation > 360:
                node.prev_orientation = node.prev_orientation - 360
            c = int(node.prev_orientation // 30)

            # If the next node is already visited, skip it
            if V[a][b][c] == 1:
                continue

            # Visualize motion
            visualize_action(cur.x, cur.y, prev_orientation, UL, RL)
                      
            plt.pause(.0001)

            # If the child node is already in the queue, compare and update the node's cost and parent as needed.
            if node_index in queue:
                if queue[node_index].cost > node.cost:
                    queue[node_index].cost = node.cost
                    queue[node_index].parent_index = cur_index
                    queue[node_index].UL = node.UL
                    queue[node_index].RL = node.RL
            else:  # Else add child to the queue.
                queue[node_index] = node

    # Backtrack the path from Goal to Start
    path_x, path_y = [goal_node.x], [goal_node.y]
    parent_index = goal_node.parent_index
    child = visited[(parent_index[0], parent_index[1], goal_node.curr_orientation)]

    visualize_action(parent_index[0], parent_index[1], goal_node.curr_orientation, goal_node.UL, goal_node.RL, color="green")

    ori = child.prev_orientation
    UL = child.UL
    RL = child.RL

    UL_list = [goal_node.UL]
    RL_list = [goal_node.RL]
    theta_list = [goal_node.prev_orientation]

    goal_node.UL_prev = child.UL
    goal_node.RL_prev = child.RL

    # Follow the parents from the goal node to the start node and add them to the path list.
    while parent_index != (start_node.x, start_node.y):
        n = visited[(parent_index[0], parent_index[1], ori)]
        path_x.append(n.x)
        path_y.append(n.y)
        UL_list.append(n.UL)
        RL_list.append(n.RL)
        theta_list.append(n.prev_orientation)
        parent_index = n.parent_index
        ori = n.curr_orientation
        
        visualize_action(parent_index[0], parent_index[1], n.curr_orientation, n.UL, n.RL, color="green")

        UL = n.UL_prev
        RL = n.RL_prev

    path_x.append(start_node.x)
    path_y.append(start_node.y)

    path_x.reverse()
    path_y.reverse()

    UL_list.append(0)
    RL_list.append(0)
    theta_list.append(start_node.prev_orientation)
    UL_list.reverse()
    RL_list.reverse()
    theta_list.reverse()

    print(UL_list)
    print(RL_list)
    print(path_x)
    print(path_y)

    for i in range(len(UL_list)): # Publish robot parameters to ROS
        UL = UL_list[i]
        RL = RL_list[i]
        theta = theta_list[i]
        move_turtlebot(UL, RL, theta)

    return path_x, path_y


def move_turtlebot(UL, RL, theta_n):
    global r, L

    # dt = 0.1
    dt = .5
    x_dot = (0.5 * r * (UL + RL) * np.cos(theta_n) * dt)
    y_dot = (0.5 * r * (UL + RL) * np.sin(theta_n) * dt)

    velocity_msg.linear.x = np.sqrt(x_dot**2 + y_dot**2)
    velocity_msg.linear.y = 0
    velocity_msg.angular.z =  (r / L) * (RL - UL) * dt

    pub.publish(velocity_msg)

    if velocity_msg.angular is 0:
        rospy.sleep(20)
    else: 
        rospy.sleep(2)

    velocity_msg.linear.x = 0
    velocity_msg.linear.y = 0
    velocity_msg.angular.z = 0
    pub.publish(velocity_msg)
    rospy.sleep(0.5)


def main():
    # set obstacle positions
    ox, oy = [], []
    scale = 0.01

    for i in np.arange(0, width, scale):
        for j in np.arange(0, height, scale):

            if .50 <= i <= 2.00 and 4.25 <= j <= 5.75:
                ox.append(i)
                oy.append(j)

            # Rectangle
            if 3.75 <= i <= 6.25 and 4.25 <= j <= 5.75:
                ox.append(i)
                oy.append(j)

            if 7.25 <= i <= 8.75 and 2.00 <= j <= 4.00:
                ox.append(i)
                oy.append(j)

            # Circle
            if (i - 2.00) ** 2 + (j - 2.00) ** 2 <= 1.00 ** 2:
                ox.append(i)
                oy.append(j)

            if (i - 2.00) ** 2 + (j - 8.00) ** 2 <= 1.00 ** 2:
                ox.append(i)
                oy.append(j)

    start_node, goal_node, step_size, RPM_left, RPM_right = begin()

    plt.xlim([0, width])
    plt.ylim([0, height])
    plt.plot(ox, oy, ".k")
    plt.grid(True)
    plt.axis("equal")

    a = [start_node.x, start_node.y]
    b = [goal_node.x, goal_node.y]

    if a != b:
        path_x, path_y = a_star(start_node, goal_node, step_size, RPM_left, RPM_right)  # Call A star algorithm

        # plt.plot(path_x, path_y, "-g")
        plt.pause(0.0001)
        plt.show()

    else:
        print("Start position equals the goal position.")


if __name__ == '__main__':
    main()

    # velocity_msg.linear.x = 0.1
    # pub.publish(velocity_msg)
    # velocity_msg.linear.y =0.1
    # velocity_msg.angular.z = 0.1

    # while rospy is not shutdown:
    #     pub.publish(velocity_msg)

"""
Update the user inputs Fixed start or RPM

"""