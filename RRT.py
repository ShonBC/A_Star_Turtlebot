# ENPM 661 - Planning for Autonomous Robots:
# Project 3 Phase 3 - RRT on Turtlebot3
# Shon Cortes, Bo-Shiang Wang

import numpy as np
import matplotlib.pyplot as plt

width = 1000
height = 1000
global robot_size
# TODO: robot size ?
robot_size = 15


# Class for storing node position, cost to come, parent index, and prev_orientation.
class Node:
    def __init__(self, x, y, cost, parent_index, prev_orientation, curr_orientation):
        self.x = x
        self.y = y
        self.cost = cost
        self.parent_index = parent_index
        self.prev_orientation = prev_orientation
        self.curr_orientation = curr_orientation


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
    # Square
    if 50 <= node[0] <= 200 and 425 <= node[1] <= 575:
        return True

    # Rectangle
    elif 375 <= node[0] <= 625 and 425 <= node[1] <= 575:
        return True

    elif 725 <= node[0] <= 875 and 200 <= node[1] <= 400:
        return True

    # Circle
    elif (node[0] - 200) ** 2 + (node[1] - 200) ** 2 <= 100 ** 2:
        return True

    elif (node[0] - 200) ** 2 + (node[1] - 800) ** 2 <= 100 ** 2:
        return True

    else:
        return False


def begin():  # Ask for user input of start and goal pos. Start and goal much be positive integers
    while True:

        start_x = 100
        start_y = 100
        # try:
        #     start_theta = int(input("Enter starting theta: "))
        # except ValueError:
        #     start_theta = 0

        goal_x = 900
        goal_y = 900
        # step_size = int(input("Enter the step size for the motion: "))

        # TODO: Remember to make start_theta and step_size to user input after testing
        start_theta = 0
        step_size = 30

        prev_orientation = start_theta

        # TODO: Remember to make RPM_left and RPM_right to user input after testing
        RPM_left = 50  # For testing program
        RPM_right = 50  # For testing program

        # Initialize start and goal nodes from node class
        start_node = Node(start_x, start_y, 0, -1, prev_orientation, prev_orientation)
        goal_node = Node(goal_x, goal_y, 0, -1, 0, 0)

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
    t = 0
    r = 5
    L = 40
    dt = 1
    Xn = Xi
    Yn = Yi
    theta_n = np.deg2rad(theta_i)
    D = 0

    Xs = Xn
    Ys = Yn
    Xn = Xi + 0.5 * r * (UL + UR) * np.cos(theta_n) * dt
    Yn = Yi + 0.5 * r * (UL + UR) * np.sin(theta_n) * dt
    theta_n = theta_i + (r / L) * (UR - UL) * dt
    D = np.sqrt((0.5 * r * (UL + UR) * np.cos(theta_n) * dt)**2
                + (0.5 * r * (UL + UR) * np.sin(theta_n) * dt)**2)

    # plot the action curve
    plt.plot([Xs, Xn], [Ys, Yn], color="blue")

    theta_n = np.rad2deg(theta_n)

    return [Xn, Yn, D, theta_n, theta_i]


def motion_model(prev_orientation, RPM_left, RPM_right, x, y):
    # TODO: determine theta angle 30? 45? (theta angle will affect the orientation_dict and V matrix)
    # theta = theta + Previous orientation
    theta = np.deg2rad(30) + np.deg2rad(prev_orientation)
    # Get the new x and y coordinates, new theta, and cost through action()
    model = [action(x, y, theta, 0, RPM_left),
             action(x, y, theta, RPM_left, 0),
             action(x, y, theta, RPM_left, RPM_left),
             action(x, y, theta, 0, RPM_right),
             action(x, y, theta, RPM_right, 0),
             action(x, y, theta, RPM_right, RPM_right),
             action(x, y, theta, RPM_left, RPM_right),
             action(x, y, theta, RPM_right, RPM_left)
             ]

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

        # If goal node is reached, Break the while loop.
        # Add a threshold(circle) for the goal node
        if (goal_node.x - cur.x) ** 2 + (goal_node.y - cur.y) ** 2 <= (1.5 * step_size) ** 2:

            goal_node.parent_index = cur.parent_index
            goal_node.cost = cur.cost
            goal_node.curr_orientation = cur.curr_orientation
            print('Goal Found')
            break

        del queue[cur_index]  # Remove the current node from the queue.
        visited[(cur.x, cur.y, cur.prev_orientation)] = cur  # Add current node to visited list.

        # Mark 1 for visited nodes in matrix V
        a = int(round(cur.x) / threshold)
        b = int(round(cur.y) / threshold)
        # c = orientation_dict[prev_orientation]
        c = prev_orientation // 30
        V[a][b][c] = 1

        # Initialize action set with prev_orientation, RPM_left, RPM_right, cur.x, cur.y
        motion = motion_model(prev_orientation, RPM_left, RPM_right, cur.x, cur.y)

        # Generate children of current node based on the action set.
        for i in range(len(motion)):
            next_x = round(cur.x + motion[i][0], 3)
            next_y = round(cur.y + motion[i][1], 3)
            child_orientation = round(motion[i][4])

            # Generate child node
            node = Node(next_x, next_y, cur.cost + motion[i][2], cur_index, child_orientation, prev_orientation)
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
            c = node.prev_orientation // 30

            # If the next node is already visited, skip it
            if V[a][b][c] == 1:
                continue

            # Visualize motion
            plt.quiver(cur.x, cur.y, motion[i][0], motion[i][1], units='xy', scale=1, color='r', width=.1)
            plt.pause(.0001)

            # If the child node is already in the queue, compare and update the node's cost and parent as needed.
            if node_index in queue:
                if queue[node_index].cost > node.cost:
                    queue[node_index].cost = node.cost
                    queue[node_index].parent_index = cur_index
            else:  # Else add child to the queue.
                queue[node_index] = node

    # Backtrack the path from Goal to Start
    path_x, path_y = [goal_node.x], [goal_node.y]
    parent_index = goal_node.parent_index
    child = visited[(parent_index[0], parent_index[1], goal_node.curr_orientation)]
    plt.quiver(child.x, child.y, goal_node.x - child.x, goal_node.y - child.y,
               units='xy', scale=1, color='r', width=.1)

    ori = child.prev_orientation
    # Follow the parents from the goal node to the start node and add them to the path list.
    while parent_index != (start_node.x, start_node.y):
        n = visited[(parent_index[0], parent_index[1], ori)]
        path_x.append(n.x)
        path_y.append(n.y)
        parent_index = n.parent_index
        ori = n.curr_orientation

    path_x.append(start_node.x)
    path_y.append(start_node.y)

    return path_x, path_y


def main():
    # set obstacle positions
    ox, oy = [], []
    for i in range(0, 1000):
        for j in range(0, 1000):

            if 50 <= i <= 200 and 425 <= j <= 575:
                ox.append(i)
                oy.append(j)

            # Rectangle
            if 375 <= i <= 625 and 425 <= j <= 575:
                ox.append(i)
                oy.append(j)

            if 725 <= i <= 875 and 200 <= j <= 400:
                ox.append(i)
                oy.append(j)

            # Circle
            if (i - 200) ** 2 + (j - 200) ** 2 <= 100 ** 2:
                ox.append(i)
                oy.append(j)

            if (i - 200) ** 2 + (j - 800) ** 2 <= 100 ** 2:
                ox.append(i)
                oy.append(j)

    start_node, goal_node, step_size, RPM_left, RPM_right = begin()

    plt.xlim([0, 1000])
    plt.ylim([0, 1000])
    plt.plot(ox, oy, ".k")
    plt.grid(True)
    plt.axis("equal")

    a = [start_node.x, start_node.y]
    b = [goal_node.x, goal_node.y]

    if a != b:
        path_x, path_y = a_star(start_node, goal_node, step_size, RPM_left, RPM_right)  # Call A star algorithm

        plt.plot(path_x, path_y, "-g")
        plt.pause(0.0001)
        plt.show()

    else:
        print("Start position equals the goal position.")


if __name__ == '__main__':
    main()