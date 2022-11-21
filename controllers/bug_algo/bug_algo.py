"""project_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Supervisor
import numpy as np
import math
from scipy.spatial.transform import Rotation as R



def theta_bound(theta):
    if theta < -np.pi:
        return np.pi-(-np.pi-theta)
    elif theta > np.pi:
        return -(np.pi-(theta-np.pi))
    return theta
    
def estimate_pose():
    global robot_pose_x,robot_pose_y,robot_pose_theta,enc, prev_l_ps, prev_r_ps, goal_theta
    c_l_ps = l_ps.getValue()
    c_r_ps = r_ps.getValue()
    error_left = c_l_ps - prev_l_ps
    error_right = c_r_ps - prev_r_ps
    dist_left = error_left*enc
    dist_right = error_right*enc
    v_ps = (dist_left + dist_right) / 2
    w_ps = (-dist_left + dist_right) / l
    prev_l_ps = c_l_ps
    prev_r_ps = c_r_ps
    robot_pose_x,robot_pose_y,robot_pose_theta = update_pose(robot_pose_x,robot_pose_y,robot_pose_theta, v_ps, w_ps)
    # print(robot_pose_x,robot_pose_y,robot_pose_theta,goal_theta)

def update_pose(x, y, theta, v, w):
    theta = theta + w
    theta = np.deg2rad(np.rad2deg(theta))
    theta = theta_bound(theta)
    x = x + v*np.cos(theta)*1
    y = y + v*np.sin(theta)*1
    return x, y, theta



def rottoeuler(orientation):
    orient = R.from_matrix(orientation)
    orient = orient.as_euler('zyx',degrees=False)
    theta = orient[0]
    return theta

def get_pose_det(ball_trans,ball_pose):
        trans = ball_trans[:3, 3:]
        rot = ball_trans[:3, :3]
        rot_inv = rot.T
        trans_inv = -rot_inv@trans
        ball_trans_robot = np.eye(4)
        ball_trans_robot[:3, :3] = rot_inv
        ball_trans_robot[:3, 3:] = trans_inv
        ball_trans_global = np.eye(4)
        ball_trans_global[:3, 3] = ball_pose[:]
        robot_pose = np.matmul(ball_trans_global, ball_trans_robot)
        x, y = robot_pose[:2, 3]
        theta = rottoeuler(robot_pose[:3, :3])
        return x,y,theta




# Meant only for verification. Function call commented in Main
def get_sim_position(robot):
    r=robot.getFromDef("e-puck")
    position = r.getPosition()
    x = position[0]
    y = position[1]
    orientation = np.array(r.getOrientation())
    orientation = np.reshape(orientation, (3,3))
    orient = R.from_matrix(orientation)
    orient = orient.as_euler('zyx',degrees=False)
    theta = orient[0]
    return x,y,theta


def cam_det(obj):
        global robot
        ball = robot.getFromId(obj[0].get_id())
        ball_pose = ball.getPosition()
        ball_trans = np.array(ball.getPose(r)).reshape((4, 4))
        return get_pose_det(ball_trans,ball_pose)
        


   
    # return T_inv



def navigation():

    global leftMotor,rightMotor,scan_list,state,robot_pose_x,robot_pose_y,robot_pose_theta,obs_pose_x,obs_pose_y,goal_x,goal_y
    flag = 0
  
    if flag > 1000:
        flag = 0
        state = 'orient'

    if state == "orient":

        if robot_pose_theta > goal_theta+0.05 or robot_pose_theta < goal_theta-0.05:
            leftMotor.setVelocity(-0.25)
            rightMotor.setVelocity(0.25)
            state = "orient"
        elif robot_pose_theta < goal_theta+0.05 and robot_pose_theta > goal_theta-0.05:
            leftMotor.setVelocity(0.0)
            rightMotor.setVelocity(0.0)
            state = "forward"
        return state


    elif state == "forward":

        if np.any(scan_list[:90] < 0.25) or np.any(scan_list[90:] < 0.25):
            state = "obstacle" 
            obs_pose_x = robot_pose_x
            obs_pose_y = robot_pose_y

        else:
            leftMotor.setVelocity(2.0)
            rightMotor.setVelocity(2.0)         
        return state


    elif state == "obstacle":
        obst_x = robot_pose_x - obs_pose_x
        obst_y = robot_pose_y - obs_pose_y
        disp = math.sqrt(obst_y**2 + obst_x**2)
        loc_1_x = start_pose_x 
        loc_1_y = start_pose_y 
        
        dest_x = goal_x
        dest_y = goal_y
        rob_pose_x = robot_pose_x
        rob_pose_y = robot_pose_y
        errx = dest_x - loc_1_x
        erry = dest_y - loc_1_y
        errxy = dest_y * loc_1_x
        erryx = dest_x * loc_1_y
        num = abs(erry * rob_pose_x - errx * rob_pose_y + erryx - errxy)
        den = math.sqrt(erry**2 + errx**2)
        
        if num/den < 0.2 and disp > 0.5:
            # print(num/den,disp)
            # print("if")
            state = 'orient'
       
        elif np.all(scan_list[75:105] > 0.25):
            # print("elif-3")
            leftMotor.setVelocity(2)
            rightMotor.setVelocity(2)
        
        elif np.any(scan_list[75:105] < 0.25):
            # print("elif-1")
            leftMotor.setVelocity(-0.25)
            rightMotor.setVelocity(0.25)
       

        
        elif np.all(scan_list[0:75] > 0.25):
            # print("elif-2")
            leftMotor.setVelocity(-0.25)
            rightMotor.setVelocity(0.25)

        # else:
            # print("else")
            # leftMotor.setVelocity(2.0)
            # rightMotor.setVelocity(2.0)


    flag+=1


if __name__ == "__main__":
    # create the Robot instance.
    robot = Supervisor()
    
    #Lidar_init
    lidar = robot.getDevice('lidar')
    lidar.enable(1)
    lidar.enablePointCloud()
    scan_list = np.array(lidar.getRangeImage())
    
    camera = robot.getDevice('camera')
    camera.enable(1)
    camera.recognitionEnable(100)
    camera.enableRecognitionSegmentation()
    
    counter =0
    # get the time step of the current world.
    timestep = 32
    
    leftMotor = robot.getDevice('left wheel motor')
    rightMotor = robot.getDevice('right wheel motor')
    
    leftMotor.setPosition(float('inf'))
    rightMotor.setPosition(float('inf'))
    
    leftMotor.setVelocity(0)
    rightMotor.setVelocity(0)

    l_ps = robot.getDevice('left wheel sensor')
    r_ps = robot.getDevice('right wheel sensor')
    l_ps.enable(timestep)
    r_ps.enable(timestep)

    #Target position
    goal_x = -2.85
    goal_y = 0.7

    #States_params_init
    state = "orient"
    test_counter = 0
    robot_pose_x, robot_pose_y = None, None
    success = False
    start_pose_x = None
    start_pose_y = None
    obs_pose_x = None  
    obs_pose_y = None  
    exe = False

###################################################
    l_ps = robot.getDevice('left wheel sensor')
    r_ps = robot.getDevice('right wheel sensor')
    l_ps.enable(timestep)
    r_ps.enable(timestep)
        
    #Robot current position
    robot_pose_x = 2.8
    robot_pose_y = 0.7
    robot_pose_theta = 0
    
    #Target position
    goal_x = -2.85
    goal_y = 0.7
    #print(goal_theta)
    
    #define parameters
    radius = 0.0205
    circumference = 2*np.pi*radius
    enc = circumference/(2*np.pi)
    
    l = 0.0568

    prev_l_ps = 0
    prev_r_ps = 0

###################################################


    while robot.step(timestep) != -1:

        print("State:",state)
        # robot_pose_x,robot_pose_y,robot_pose_theta = get_sim_position(robot) 
        goal_theta = np.arctan2(goal_y - robot_pose_y, goal_x - robot_pose_x)     
        # print(robot_pose_x,robot_pose_y,robot_pose_theta,goal_theta)

        # print(goal_theta,robot_pose_theta)
        test_counter+=1
        scan_list = np.array(lidar.getRangeImage())
        # print(scan_list[0:5],scan_list[90:95],scan_list[170:175])
        
        estimate_pose()
        # pose_callback stuff
        error_x = robot_pose_x - goal_x
        error_y = robot_pose_y - goal_y
        goal_distance = math.sqrt(error_y**2 + error_x**2)
        # print(goal_distance)
        if goal_distance <= 0.4:
            success = True
        # print(success)    
        start_pose_x = robot_pose_x
        start_pose_y = robot_pose_y
        
        r=robot.getFromDef("e-puck")
        recObjs = camera.getRecognitionObjects()
        recObjsNum = camera.getRecognitionNumberOfObjects()
        if recObjsNum != 0:  
            x,y,theta = cam_det(recObjs)
            robot_pose_x,robot_pose_y,robot_pose_theta =  x,y,theta
        if not success:
            navigation()
            
        else:    
            state = "goal_reached"
        if state == "goal_reached":
            leftMotor.setVelocity(0.0)
            rightMotor.setVelocity(0.0)  
