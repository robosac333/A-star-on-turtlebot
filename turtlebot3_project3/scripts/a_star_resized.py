#!/usr/bin/env python
# coding: utf-8


import numpy as np
import cv2
import matplotlib.pyplot as plt
import heapq
import math
from matplotlib.animation import FuncAnimation
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration

'''
To move the turtlebot on ROS2 Node
'''
def move_turtlebot(velocity_list):
    rclpy.init()

    node = rclpy.create_node('turtlebot_controller')

    publisher = node.create_publisher(Twist, 'cmd_vel', 10)

    msg = Twist()
    linear_velocity = msg.linear.x  
    angular_velocity = msg.angular.z

    r = 0.33
    L = 0.306
    dt = 1
    
    while rclpy.ok():
        # node.get_logger().info('Moving forward... Velocity List: {}'.format(velocity_list))        
        for move in velocity_list:
            linear_velocity, angular_velocity = calculate_velocities(move[0], move[1], r, L)
            # time_sleep = dt
            # msg.angular.z = angular_velocity
            # publisher.publish(msg)
            # time.sleep(time_sleep)
            # msg.linear.x = linear_velocity
            # publisher.publish(msg)
            # time.sleep(time_sleep)
            node.get_logger().info('Publishing velocity: Linear - {}, Angular - {}, Move - {}'.format(linear_velocity, angular_velocity, move))       
            # # D = move[2]
            # node.get_logger().info('Time sleep was: {}'.format(D/linear_velocity))
            # # time.sleep(linear_velocity/D)
            # time.sleep(D/linear_velocity)
            now = node.get_clock().now()
            duration = Duration(seconds=0.1)
            end_time = now + duration
            while(node.get_clock().now() < end_time):
                    msg.linear.x = linear_velocity
                    
                    msg.angular.z = angular_velocity
                    publisher.publish(msg)
                
        break
    msg.linear.x = 0.0
    msg.angular.z = 0.0
    publisher.publish(msg)
    node.get_logger().info('Stopping the robot...Goal Reached 2')
    node.destroy_node()
    rclpy.shutdown()

'''
Calculates the linear and angular velocities of the robot given the RPMs of the left and right wheels
'''
def calculate_velocities(rpm_left, rpm_right, wheel_radius, wheelbase):
    # Convert RPM to radians per second
    omega_left = rpm_left * (2 * 3.14 / 60)
    omega_right = rpm_right * (2 * 3.14 / 60)
    # omega_left = rpm_left
    # omega_right = rpm_right
    # Calculate linear velocity
    # linear_velocity = ((omega_left + omega_right) / 2) * (2 * math.pi * wheel_radius)
    linear_velocity = ((omega_left + omega_right) / 2)
    # Calculate angular velocity
    angular_velocity = ((omega_left - omega_right) / wheelbase) * wheel_radius

    return linear_velocity, angular_velocity

'''
To return all the neighbours after a move
'''
def possible_moves(tup , step_size, RPM1, RPM2):
    Xi, Yi, Thetai = tup
    move_list = [(0, RPM1), (RPM1, 0), (RPM1, RPM1), (0, RPM2), (RPM2, 0), (RPM2, RPM2), (RPM1, RPM2), (RPM2, RPM1)]

    moves = []
    
    # for move in move_list:
    #     UL,UR= move
    #     #t = 0
    #     r = 0.33
    #     L = 0.306
    #     dt = 1
    #     UL = 3.14 * (UL / 30)
    #     UR = 3.14 * (UR / 30)
    #     D = 0

    #     Thetan = Thetai + (r / L) * (UR - UL) * dt
    #     if (Thetan == 0):
    #         Thetan = 0
    #     elif (Thetan == 360):
    #         Thetan = 360
    #     else :
    #        Thetan = Thetan % 360
    #     Xn = Xi + 0.5*r * (UL + UR) * np.cos(np.radians(Thetan)) * dt
    #     Yn = Yi +  0.5*r * (UL + UR) * np.sin(np.radians(Thetan)) * dt
    #     D = math.sqrt(math.pow((Xn - Xi), 2) + math.pow((Yn - Yi), 2))

           
    #     moves.append((Xn,Yn, Thetan, UL, UR, D))
    for move in move_list:
        UL, UR = move
        t = 0
        r = 0.105
        L = 0.16
        dt = 0.1

        UL = 3.14 * (UL / 30)
        UR = 3.14 * (UR / 30)
        # ang_vel = (r / L) * (UR - UL)
        # lin_vel = 0.5 * r * (UL + UR)
        newI = Xi
        newJ = Yi
        newTheta = 3.14 * Thetai / 180
        D = 0

        while t < 1:
            t = t + dt
            Delta_Xn = 0.5 * r * (UL + UR) * math.cos(newTheta) * dt
            Delta_Yn = 0.5 * r * (UL + UR) * math.sin(newTheta) * dt
            newI += Delta_Xn
            newJ += Delta_Yn
            newTheta += (r / L) * (UR - UL) * dt
            D = D + math.sqrt(math.pow(Delta_Xn, 2) + math.pow(Delta_Yn, 2))
        newTheta = 180 * newTheta / 3.14

        if newTheta > 0:
            newTheta = newTheta % 360
        elif newTheta < 0:
            newTheta = (newTheta + 360) % 360

        newI = getRoundedNumber(newI)
        newJ = getRoundedNumber(newJ)
        moves.append((newI, newJ, newTheta, UL, UR, D))

    return moves


def getRoundedNumber(i):
    i = 50 * i
    i = int(i)
    i = float(i) / 50.0
    return i
'''
To check if theta is less than the threshold
'''
def is_in_check(tup , visited):
    x , y , theta = tup
    if ((x , y)) in visited :
            thetas = visited[x, y]
            for theta_c in thetas:
                if abs(theta_c - theta) < 30:
                    return True
    return False


'''
To check if the move is legal or not
'''
def is_move_legal(tup , img_check, total_clearence, scale):
    x , y = tup
    #pixel_value = img_check[y, x]
#     pixel_value = tuple(pixel_value)
    if x < int(total_clearence/scale)  or x >= int((6000 - total_clearence)/scale) or y < int(total_clearence/scale) or y >= int((2000 - total_clearence)/scale):
        return False
    elif tuple(img_check[int(round(y)), int(round(x))]) == (255 , 255 , 255) :
        #print(f"Point {point} is in the free region.(here 6)")
        return False
    else :
        return True

'''
Applying A* Algorithm
'''

def algorithm(start , goal, step_size, image, total_clearence, scale) :
    #Please note that in the line that follows you will need to change the video driver to an 
    #appropriate one that is available on your system.
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    ##Please change the path to the desired path.
    out = cv2.VideoWriter(r'test.mp4', fourcc, 20, (6000, 2000))
    
    x_start , y_start , theta_start = start
    
    x_goal , y_goal , theta_goal = goal
    
    h = math.sqrt((x_start - x_goal)**2 + (y_start - y_goal)**2)
    
    queue_open =  [(h , (start , 0))]

    heapq.heapify(queue_open)

    visited = {}
    visited_parent = {}
    path = []
    info_dict = {start : ((None , None) , 0 , 0 , 0, 0)}
    # frame_list = []
    ite = 0
    move_list = []
    reached = 0
    velocity_list = []
    while queue_open :
        element = heapq.heappop(queue_open)
        t_c , tup = element
        node , c_2_c = tup
#         print(node)
        info = info_dict[node]
        parent , c_2_c_p, UL, UR, D = info
        visited_parent[node] = (parent , UL , UR, D)
        x , y , theta = node
        theta = theta % 360
        x_int = int(round(x))
        y_int = int(round(y))
        #node = (x_int , y_int , theta)
        #figure out where exactly to store the theta value
        if (x_int , y_int) in visited :
            thetas = visited[x_int , y_int]
            thetas.append(theta)
        else:
            thetas = []
            thetas.append(theta)
            visited[x_int , y_int] = thetas
#         visited_parent[node] = parent
        if (math.sqrt((node[0] - goal[0])**2 + (node[1] - goal[1])**2) <= 1.5)  :
            #print(reached)
            path.append(node)
            #velocity_list = []
            while node != start :
                parent , UL , UR, D = visited_parent[node]
                velocity_list.append((UL , UR, D))
                path.append(parent)
                node = parent
            velocity_list.append((0 , 0, 0))
            path.reverse()
            velocity_list.reverse()
            #print(path)
            for point in (path) :
                x, y, theta = point
                x_int = int(round(x))
                y_int = int(round(y))
                cv2.circle(img_ori , (x_int , y_int) , 1 , (0 , 255 , 0) , -1)
            print('reached')
            img_ori_copy = img_ori.copy()
            flipped_vertical = cv2.flip(img_ori_copy, 0)
            for i in range(100):
                out.write(flipped_vertical)
            reached = 1
            break

        moves = possible_moves((x_int , y_int , theta) , step_size, RPM1, RPM2)
        for m_v in (moves) :
            x , y, theta, UL, UR, D = m_v
            move = (x , y ,theta)
            x_int = int(round(x))
            y_int = int(round(y))
            Bool1 = is_move_legal((x_int , y_int) , image, total_clearence, scale)
            #move = (x_int , y_int , theta)
            if (Bool1 == True):
                #print(move)
                Bool2 = is_in_check((x_int , y_int , theta) , visited)
                if (Bool2 == False):
                    cv2.circle(img_ori , (x_int , y_int) , 1 , (255 , 0 , 0) , -1)
                    move_list.append((move[0] , move[1]))
                    if (ite % 10000) == 0 :
                            img_ori_copy = img_ori.copy()
                            flipped_vertical = cv2.flip(img_ori_copy, 0)
                            out.write(flipped_vertical)
                    if move in info_dict :
                        info = info_dict[move]
                        parent , c_2_c_p, UL_p , UR_p, D = info
                        c_2_c_n = c_2_c + 1
                        if (c_2_c_n < c_2_c_p):
                            #total cost calculation
                            total_cost = c_2_c_n + math.sqrt((move[0] - goal[0]) ** 2 + (move[1] - goal[1]) ** 2)
                            info_dict[move] = (node , c_2_c_n, UL, UR, D)
                            queue_open = [(k, v) for k, v in queue_open if v[0] != move]
                            heapq.heapify(queue_open)
                            heapq.heappush(queue_open , (total_cost , (move , c_2_c_n)))
                    elif move not in info_dict :
                        total_cost = (c_2_c + 1) + math.sqrt((move[0] - goal[0]) ** 2 + (move[1] - goal[1]) ** 2)
                        info_dict[move] = (node , c_2_c + 1,  UL, UR, D)
                        heapq.heappush(queue_open , (total_cost , (move , c_2_c + 1)))
        ite += 1

    out.release()
    return visited_parent, reached, path, move_list, velocity_list

'''
Function to plot all the Visited Nodes
'''
def animate_search(visited, circle_center, scale):
    fig, ax = plt.subplots(figsize=(9, 3)) #set animate to 12:5 match map shape
    ax.set_xlim(0, 6000/scale) #set animate x axis
    ax.set_ylim(0, 2000/scale) #set animate y axis

    #show obstacles
    for polygons in obstacles:
        polygon = plt.Polygon(polygons, facecolor="red", edgecolor='black')
        ax.add_patch(polygon)

    # Draw circle
    circle = plt.Circle(circle_center, radius=600/scale, color='red', alpha=0.5)  # Adjust radius as needed
    ax.add_artist(circle)

    points = ax.scatter([], [], s=1, color='blue') 

    def init():
        points.set_offsets(np.empty((0, 2))) 
        return points,

    def update(frame):
        skip = 50000 #set flames skip
        frame *= skip 
        visited_points = np.array(visited[:frame+1])
        points.set_offsets(visited_points)
        return points,

    ani = FuncAnimation(fig, update, frames=len(visited), init_func=init, blit=True, interval=1)
    plt.show()

'''
Animate the path
'''
def animate_path(path, circle_center, scale):
    fig, ax = plt.subplots(figsize=(9,3))
    ax.set_xlim(0, int(6000/scale))
    ax.set_ylim(0, int(2000/scale))

    for polygons in obstacles:
        polygon = plt.Polygon(polygons, facecolor="gray", edgecolor='black')
        ax.add_patch(polygon)

    # Draw circle
    circle = plt.Circle(circle_center, radius=600/scale, color='black', alpha=0.5)  # Adjust radius as needed
    ax.add_artist(circle)

    line, = ax.plot([], [], 'b-', lw=2)  # Path line

    def init():
        line.set_data([], [])
        return line,

    def update(frame):
        skip = 20 #set flames skip
        frame *= skip
        x, y = zip(*path[:frame+1]) #get path
        line.set_data(x, y)
        return line,

    ani = FuncAnimation(fig, update, frames=len(path), init_func=init, blit=True, interval=50)
    plt.show()



if __name__ == '__main__':
    scale = 1000

    circle_center = (int(4200/scale), int(1200/scale))

    # obstacles = [
    #     [(int(1500/scale), int(1000/scale)), (int(1500/scale), int(2000/scale)), (int(1750/scale), int(2000/scale)), (int(1750/scale), int(1000/scale))],       
    #     [(int(2500/scale), int(0/scale)), (int(2500/scale), int(1000/scale)), (int(2750/scale), int(1000/scale)), (int(2750/scale), int(0/scale))]
    # ]

    obstacles = [
        [((1500/scale), (1000/scale)), ((1500/scale), (2000/scale)), ((1750/scale), (2000/scale)), ((1750/scale), (1000/scale))],       
        [((2500/scale), (0/scale)), ((2500/scale), (1000/scale)), ((2750/scale), (1000/scale)), ((2750/scale), (0/scale))]
    ]
    '''
    First Plotting the Bloated Figure
    '''
    # step_size = float((input("Enter the step size: ")))
    # radius = int(input("Enter the radius of the robot: "))
    # cle = int(input("Enter the clearence: "))
    step_size = 10
    radius = int(230/scale)
    cle = int(5/scale)
    total_clearence = int((cle + radius)/scale)

    # Coordinates of the first polygon
    x1_polygon1, x2_polygon1, y1_polygon1, y2_polygon1 = int((1500 - total_clearence)/scale) , int((1750 + total_clearence)/scale) , int((1000 - total_clearence)/scale) , int(2000/scale)

    # Coordinates of the second polygon
    x1_polygon2, x2_polygon2, y1_polygon2, y2_polygon2 = int((2500 - total_clearence)/scale) , int((2750 + total_clearence)/scale) , int(0/scale) , int((1000 + total_clearence)/scale)

    # Create a blank image with size 1190x490
    img_check = np.zeros((int(2000/scale), int(6000/scale) , 3), dtype=np.uint8)

    # Define the vertices of the first polygon
    pts_polygon1 = np.array([[x1_polygon1, y1_polygon1], [x2_polygon1, y1_polygon1], [x2_polygon1, y2_polygon1], [x1_polygon1, y2_polygon1]], np.int32)
    pts_polygon1 = pts_polygon1.reshape((-1, 1, 2))

    # Define the vertices of the second polygon
    pts_polygon2 = np.array([[x1_polygon2, y1_polygon2], [x2_polygon2, y1_polygon2], [x2_polygon2, y2_polygon2], [x1_polygon2, y2_polygon2]], np.int32)
    pts_polygon2 = pts_polygon2.reshape((-1, 1, 2))


    # Fill the first polygon with white color
    cv2.fillPoly(img_check, [pts_polygon1], (255 , 255 , 255))

    # Fill the second polygon with white color
    cv2.fillPoly(img_check, [pts_polygon2], (255 , 255 , 255))

    # Draw a circle centered at (4200, 1200) with radius 600
    circle_center = (int(4200/scale), int(1200/scale))
    circle_radius = int((600 + total_clearence)/scale)
    cv2.circle(img_check, circle_center, circle_radius, (255, 255, 255), -1)

    '''
    Now Plotting the Maze on top of the bloated figure
    '''

    # Coordinates of the first polygon
    x1_polygon1, x2_polygon1, y1_polygon1, y2_polygon1 = int(1500/scale) , int(1750/scale) , int(1000/scale) , int(2000/scale)
    
    # Coordinates of the second polygon
    x1_polygon2, x2_polygon2, y1_polygon2, y2_polygon2 = int(2500/scale), int(2750/scale), int(0/scale), int(1000/scale)

    # Create a blank image with size 1190x490
    img_ori = np.zeros((int(2000/scale), int(6000/scale) ,3), dtype=np.uint8)

    # Define the vertices of the first polygon
    pts_polygon1 = np.array([[x1_polygon1, y1_polygon1], [x2_polygon1, y1_polygon1], [x2_polygon1, y2_polygon1], [x1_polygon1, y2_polygon1]], np.int32)
    pts_polygon1 = pts_polygon1.reshape((-1, 1, 2))

    # Define the vertices of the second polygon
    pts_polygon2 = np.array([[x1_polygon2, y1_polygon2], [x2_polygon2, y1_polygon2], [x2_polygon2, y2_polygon2], [x1_polygon2, y2_polygon2]], np.int32)
    pts_polygon2 = pts_polygon2.reshape((-1, 1, 2))

    # Fill the first polygon with white color
    cv2.fillPoly(img_ori, [pts_polygon1], (255 , 255 , 255))

    # Fill the second polygon with white color
    cv2.fillPoly(img_ori, [pts_polygon2], (255 , 255 , 255))

    # Draw a circle centered at (4200, 1200) with radius 600
    circle_center = (int(4200/scale), int(1200/scale))
    circle_radius = int(600/scale)
    cv2.circle(img_ori, circle_center, circle_radius, (255 , 255, 255), -1)

    '''
    Initialzation Parameters
    '''
    start_x = 500/scale
    start_y = 1000/scale
    start_theta = 0
    # goal_x = 4000
    # goal_y = 1780
    goal_x = 1000/scale
    goal_y = 1000/scale
    RPM1 = 100
    RPM2 = 100
    goal_theta = start_theta

    start = (start_x , start_y, start_theta)
    goal = (goal_x , goal_y, goal_theta)
    Bool1 = is_move_legal((start[0] , start[1]) , img_check, total_clearence, scale)
    Bool2 = is_move_legal((goal[0] , goal[1]) , img_check, total_clearence, scale)

    if Bool1 == True and Bool2 == True :
        print('correct positions entered algo is executing')
        visited_parent , reached, path, move_list, velocity_list = algorithm (start , goal, step_size, img_check, total_clearence, scale)
        if reached == 1 :
            print('Path is available')
        else  :
            print('Did not reach')
    else :
        print('please run the code cell again and enter valid start and goal positions')


    '''
    Storing all the path coordinates in a list
    '''
    node = path[0]
    while node != start :
        parent = visited_parent[node]
        path.append(parent)
        node = parent
    # path.reverse()
    #print(path)
    # print(velocity_list)
    coord_list = []
    for point in path :
        x , y , theta = point
        coord_list.append((x , y))

    # animate_search(move_list, circle_center)

    animate_path(coord_list, circle_center, scale)

    # move_turtlebot(velocity_list)