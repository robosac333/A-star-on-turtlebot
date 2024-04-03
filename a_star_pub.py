
import heapq
import time
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import math
from matplotlib.animation import FuncAnimation


def check_for_rect(x, y):
    return (x >= 1500/1000 - total_clearance and x <= 1750/1000 + total_clearance) and (y >= 1000/1000 - total_clearance and y <= 2000/1000)

def check_for_rect1(x, y):
    return (x >= 2500/1000 - total_clearance and x <= 2750/1000 + total_clearance) and (y >= 0 and y <= 1000/1000 + total_clearance)

def check_for_circle(x, y):
    return math.sqrt((x - 4200/1000)**2 + (y - 1200/1000)**2) <= 600/1000 + total_clearance

def check_for_maze(x, y):
    return (x < total_clearance or x > 5999/1000 - total_clearance) or (y < total_clearance or y > 1999/1000 - total_clearance)

def is_move_legal(x, y):
    if check_for_rect(x, y) or check_for_circle(x, y) or check_for_rect1(x, y) or check_for_maze(x, y):
        # print("The point is in the obstacle space")
        return False
    else:
        return True

'''
Ask the user for the initial and goal points
'''
def give_inputs():
    x_initial = int(input("Provide the initial x coordinate: "))
    y_initial = int(input("Provide the initial y coordinate: "))
    thetas = int(input("Provide the thetas value: "))

    x_goal = int(input("Provide the goal x coordinate: "))
    y_goal = int(input("Provide the goal y coordinate: "))
    if is_move_legal(x_initial, y_initial) and is_move_legal(x_goal, y_goal):
        print("The initial and goal points are in the free space. Executing algorithm")
    else :
        print("The initial and goal points are in the obstacle space. Please provide different points.")
        return give_inputs()
    return x_initial, y_initial, thetas, x_goal, y_goal


def possible_moves(tup , step_size, RPM1, RPM2):
    Xi, Yi, Thetai = tup
    RPM1 = 10
    RPM2 = 10
    move_list = [(0, RPM1), (RPM1, 0), (RPM1, RPM1), (0, RPM2), (RPM2, 0), (RPM2, RPM2), (RPM1, RPM2), (RPM2, RPM1)]

    moves = []
    
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
            # D = D + math.sqrt(math.pow(Delta_Xn, 2) + math.pow(Delta_Yn, 2))
        newTheta = 180 * newTheta / 3.14

        if newTheta > 0:
            newTheta = newTheta % 360
        elif newTheta < 0:
            newTheta = (newTheta + 360) % 360

        newI = getRoundedNumber(newI)
        newJ = getRoundedNumber(newJ)
        moves.append((newI, newJ, newTheta, UL, UR))


    return moves

def getRoundedNumber(i):
    i = 50 * i
    i = int(i)
    i = float(i) / 50.0
    return i

def is_in_check(tup , visited):
    x , y , theta = tup
    if ((x , y)) in visited :
            thetas = visited[x, y]
            for theta_c in thetas:
                if abs(theta_c - theta) < 30:
                    return True
    return False


def algorithm(start , goal, step_size, total_clearence) :
    x_start , y_start , theta_start = start
    
    x_goal , y_goal , theta_goal = goal
    
    h = math.sqrt((x_start - x_goal)**2 + (y_start - y_goal)**2)
    
    queue_open =  [(h , (start , 0))]

    heapq.heapify(queue_open)

    visited = {}
    visited_parent = {}
    path = []
    info_dict = {start : ((None , None) , 0, 0, 0)}
    # frame_list = []
    iteration = 0
    move_list = []
    visited_nodes = []
    velocity_list = []
    reached = 0
    while queue_open :
        element = heapq.heappop(queue_open)
        t_c , tup = element
        node , c_2_c = tup
        # print(node)
        info = info_dict[node]
        parent , c_2_c_p, UL, UR = info
        visited_parent[node] = (parent, UL, UR)
        print(node)
        x , y , theta = node
        theta = theta % 360
#         x_int = int(round(x))
#         y_int = int(round(y))
        #node = (x_int , y_int , theta)
        #figure out where exactly to store the theta value
        if (x , y) in visited :
            thetas = visited[x , y]
            thetas.append(theta)
        else:
            thetas = []
            thetas.append(theta)
            visited[x , y] = thetas
            # visited_parent[node] = parent
        if (math.sqrt((node[0] - goal[0])**2 + (node[1] - goal[1])**2) <= 1.5)  :
            print(reached)
            path.append(node)

            while node != start :
                parent, UL, UR = visited_parent[node]
                velocity_list.append((UL, UR))
                path.append(parent)
                node = parent
            
            velocity_list.append((0, 0))
            path.reverse()
            velocity_list.reverse()
            # print(path)
            return visited_parent, reached, path, move_list, velocity_list
            # animate_path(coord_list, circle_center)
            # Plot the path nodes
            # for node in path:
            #     ax.plot(node[0], node[1], 'ro', alpha=0.3, markersize=1)

        moves = possible_moves((x , y , theta) , step_size, RPM1, RPM2)
        for m_v in (moves) :
            x , y, theta, UL, UR = m_v
            move = (x , y , theta)
#             x_int = int(round(x))
#             y_int = int(round(y))
            Bool1 = is_move_legal(x , y)
            #move = (x_int , y_int , theta)
            if (Bool1 == True):
                #print(move)
                Bool2 = is_in_check((x , y , theta) , visited)
                if (Bool2 == False):
                    move_list.append((move[0] , move[1]))

                    # Add the visited node to the list
                    visited_nodes.append((x, y))


                    # Plotting the visited nodes
                    # if iteration % 3000 == 0:  # Plot every 100th node
                    #     visited_x = [node[0] for node in visited_nodes]
                    #     visited_y = [node[1] for node in visited_nodes]
                    #     ax.plot(visited_x, visited_y, 'go', alpha=0.3, markersize=0.2)
                    #     plt.pause(0.01)

                    if move in info_dict :
                        info = info_dict[move]
                        parent , c_2_c_p, UL_p, UR_p = info
                        c_2_c_n = c_2_c + 1
                        if (c_2_c_n < c_2_c_p):
                            #total cost calculation
                            total_cost = c_2_c_n + math.sqrt((move[0] - goal[0]) ** 2 + (move[1] - goal[1]) ** 2)
                            info_dict[move] = (node , c_2_c_n, UL, UR)
                            queue_open = [(k, v) for k, v in queue_open if v[0] != move]
                            heapq.heapify(queue_open)
                            heapq.heappush(queue_open , (total_cost , (move , c_2_c_n)))
                    elif move not in info_dict :
                        total_cost = (c_2_c + 1) + math.sqrt((move[0] - goal[0]) ** 2 + (move[1] - goal[1]) ** 2)
                        info_dict[move] = (node , c_2_c + 1, UL, UR)
                        heapq.heappush(queue_open , (total_cost , (move , c_2_c + 1)))
        iteration += 1

    #out.release()
    return visited_parent, reached, path, move_list, velocity_list

'''
Animate the path
'''
def animate_path(path, circle_center):
    fig, ax = plt.subplots(figsize=(9,3))
    ax.set_xlim(0, 6000/1000)
    ax.set_ylim(0, 2000/1000)

    for polygons in obstacles:
        polygon = plt.Polygon(polygons, facecolor="gray", edgecolor='black')
        ax.add_patch(polygon)

    # Draw circle
    circle = plt.Circle(circle_center, radius=600/1000, color='black', alpha=0.5)  # Adjust radius as needed
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

if __name__ == "__main__":
    '''
    Set the time to 0
    '''
    start_time = time.time()
    
    '''
    Defining the Environment
    '''
    # create a figure and axis object
    # fig, ax = plt.subplots(figsize=(7,2.5))

    # create a Rectangle object
    rect = patches.Rectangle((1500/1000, 1000/1000), 250/1000, 1000/1000, linewidth=1, edgecolor='r', facecolor='none')
    rect1 = patches.Rectangle((2500/1000, 0), 250/1000, 1000/1000, linewidth=1, edgecolor='r', facecolor='none')

    # create a Circle object
    circle = patches.Circle((4200/1000, 1200/1000), 600/1000, linewidth=1, edgecolor='b', facecolor='none')

    '''
    Checking for Obstacles
    '''
    robot_radius = 220
    clearance = 0
    total_clearance = (robot_radius + clearance) / 1000

    '''
    Plotting the Environment
    '''
    # ax.add_patch(rect)
    # ax.add_patch(rect1)
    # ax.add_patch(circle)

    '''
    Write the parameters of the environment
    '''
    # start_x, start_y, start_theta, goal_x, goal_y = give_inputs()
    iteration = 0
    RPM1 = 10
    RPM2 = 10
    step_size = 1
    start_x = 500/1000
    start_x = round(start_x , 4)
    start_y = 1000/1000
    start_y = round(start_y , 4)
    start_theta = 0
    # goal_x = 950
    # goal_y = 300
    # goal_x = 100
    # goal_y = 20
    goal_x = 5750/1000
    goal_x = round(goal_x , 4)
    goal_y = 1000/1000
    goal_y = round(goal_y , 4)
    goal_theta = start_theta

    start = (start_x , start_y, start_theta)
    goal = (goal_x , goal_y, goal_theta)

    visited_parent , reached, path, move_list, velocity_list = algorithm (start , goal, step_size, total_clearance)
    #print(path)
    print(velocity_list)
    # Set axis labels and limits
    # ax.set_xlabel('X-axis $(m)$')
    # ax.set_ylabel('Y-axis $(m)$')

    # ax.set_xlim(0, 6000)
    # ax.set_ylim(0, 2000)

    # Plot the initial and goal points
    # ax.plot(start_x, start_y, 'bo', label='Initial Point')
    # ax.plot(goal_x, goal_y, 'ro', label='Goal Point')

    # Record the end time
    end_time = time.time()

    # Print the time taken to find the path
    print("Time taken to find the path:", (end_time - start_time)/60, "minutes")

    obstacles = [
        [(1500/1000, 1000/1000), (1500/1000, 2000/1000), (1750/1000, 2000/1000), (1750/1000, 1000/1000)],

        [(2500/1000, 0), (2500/1000, 1000/1000), (2750/1000, 1000/1000), (2750/1000, 0)]
    ]


    circle_center = (4200/1000, 1200/1000)

    coord_list = []
    for point in path :
        x , y , theta = point
        coord_list.append((x , y))

    animate_path(coord_list, circle_center)

    # plt.show()