#!/usr/bin/env python

import rospy 
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
import numpy as np
from numba import jit
from time import time


global map,res,height,width,arr,r


print('Starting...')
rospy.init_node('aStar', anonymous=True)
map = rospy.wait_for_message("/map",OccupancyGrid)
res = map.info.resolution
height = map.info.height
width = map.info.width
conv = np.array([[1,1,1,1,1],[1,2,2,2,1],[1,2,10,2,1],[1,2,2,2,1],[1,1,1,1,1]])
r = rospy.Rate(1)
pub = rospy.Publisher('/path_to_goal',Path,queue_size=10)



@jit(nopython = True)
def convol(arr,conv):
    x = arr.shape[0]
    y = arr.shape[1]
    
    temp = np.ones((x+4,y+4))*100
    temp[2:x+2,2:y+2] = arr.copy()
    t2 = np.ones((x,y))*100

    for i in range(2,x):
        for j in range(2,y):
            t2[i-2,j-2] = np.sum(temp[i-2:i+3,j-2:j+3]*conv)/np.sum(conv)

    return t2


def extrapolate(conv,data,val = 5):
    a= np.array(data.data).reshape(height,width).T[1511:1825,1322:2010]
    for i in range(val):
        a[a <= 5] = 5
        a = convol(a,conv)
    a[a >= 95] = 1000
    return a


arr = extrapolate(conv,map)
print('Map Ready!')

def check(x,y,arr):
    return (x < arr.shape[0]) and (x >= 0) and (y < arr.shape[1]) and (y >= 0)


def get_path(ip, gp, predecessors):
    global route
    route = Path()
    route.header.seq = 1
    route.header.stamp = rospy.Time.now()
    route.header.frame_id = 'map'
    
    key = gp
    path = [gp]
    
    while (key != ip):
        po = PoseStamped()
        po.header = route.header
        po.pose.position.x = res*(key[0] - arr.shape[0]/2 )
        po.pose.position.y = res*(key[1] - arr.shape[1]/2 )

        key = predecessors[key]
        # print(key)

        route.poses.insert(0,po)
        path.insert(0, key)  
    return path,route

# Returns outgoing edges 
def out_edges(arr, cp):

    out = {}
    x = cp[0]
    y = cp[1]


    if check(x,y-1,arr):
        out[(x,y-1)] = arr[x][y-1]

    if check(x+1,y,arr):
        out[(x+1,y)] = arr[x+1][y]

    if check(x+1,y-1,arr):
        out[(x+1,y-1)] = arr[x+1][y-1]*2**0.5

    if check(x,y+1,arr):
        out[(x,y+1)] = arr[x][y+1]

    if check(x-1,y+1,arr):
        out[(x-1,y+1)] = arr[x-1][y+1]*2**0.5
    
    if check(x-1,y-1,arr):
        out[(x-1,y-1)] = arr[x-1][y-1]*2**0.5

    if check(x-1,y,arr):
        out[(x-1,y)] = arr[x-1][y]*2**0.5

    if check(x+1,y+1,arr):
        out[(x+1,y+1)] = arr[x+1][y+1]*2**0.5

    return out
    
    

# Returns heuristic from current pose to goal pose
def heu(cp, gp):
    # print(((cp[0] - gp[0])**2 + (cp[1] - gp[1])**2)**0.5)
    return (((cp[0] - gp[0])**2 + (cp[1] - gp[1])**2)**0.5)


# Returns key with smallest cost

def pop_smallest(open_pose):
    val = np.array(open_pose.values())
    min = np.argmin(val)

    return list(open_pose.keys())[min], np.min(val)

# A* path planner 

def planner(arr,ip, gp):
    it = 0
    open_pose = {}
    closed_pose = {}
    predecessors = {}


    costs = {}
    costs[ip] = 0

    open_pose[ip] = costs[ip] + heu(ip,gp)
    

    while len(open_pose) > 0:
        cp,cc = pop_smallest(open_pose)

        if gp == cp:
            print('Path Found!')
            break

        else:
        
            edges = out_edges(arr, cp)
            # print(cp)

            for edge in zip(edges.keys(), edges.values()):
                it += 1

                if edge[0] in closed_pose.keys():
                    continue

                if edge[0] not in open_pose.keys() :
                    costs[edge[0]] = edge[1] + costs[cp]
                    open_pose[edge[0]] = costs[edge[0]] + heu(edge[0],gp) 
                    predecessors[edge[0]] = cp
                    

                elif (edge[1] + costs[cp] + heu(edge[0],gp) < open_pose[edge[0]]):
                    costs[edge[0]] = edge[1] + costs[cp]
                    open_pose[edge[0]] = costs[edge[0]] + heu(edge[0],gp) 
                    predecessors[edge[0]] = cp
                        
            
            cp , cc = pop_smallest(open_pose)
            open_pose.pop(cp)

            closed_pose[cp] = cc
    # print(predecessors)
    return get_path(ip,gp,predecessors)



def pose_cb(data,goal):  
    global pub
    print('Goal Received')
    x = int(data.pose.pose.position.x/res + arr.shape[0]/2)
    y = int(data.pose.pose.position.y/res + arr.shape[1]/2)

    init_pose = (x,y)

    x = int(goal.pose.position.x/res + arr.shape[0]/2)
    y = int(goal.pose.position.y/res + arr.shape[1]/2)
    goal_pose = (x,y)

    print('Value = ',arr[init_pose],arr[goal_pose])

    begin = time()
    print('Planning Started')
    pub.publish(planner(arr,init_pose, goal_pose)[1])
    end = time()
    print('Time = ', end - begin)
    
    r.sleep()


def main():
    odomSub = Subscriber('/ground_truth/state',Odometry)
    goalSub = Subscriber('/move_base_simple/goal',PoseStamped)


    ats = ApproximateTimeSynchronizer(
    [odomSub, goalSub], 100, 100, allow_headerless=True
    )
    ats.registerCallback(pose_cb)
    rospy.spin()

if __name__ == '__main__':
    main()
    