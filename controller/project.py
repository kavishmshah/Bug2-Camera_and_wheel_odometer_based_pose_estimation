"""hw2_controller controller."""

from controller import Robot, Supervisor
from controller import CameraRecognitionObject
import numpy as np
import math

# constants
fov = 0.84 # the field of view of the camera (rad)
w = 640 # the width of camera (pixels)

# Task 1:
def posOfImgToBearing(x,w,fov): 
    # TODO: Calculate the bearing measurement based on landmark's position on image
    d = (1/2 * w)/np.tan(fov/2)
    term = (w/2-x)/ d
    theta = np.arctan(term)
    # print("Theta:",theta)
    return theta


    
def threeLandmarksBearing(thetas,G_p_Lis): 
    # TODO: Estimate the robot's 2D position. Return 3*1 nparray(s) (x, y, θ) 
        
        #Using Eq 5.47, calculate u
        G_p_Lis = np.array(G_p_Lis)
        c = np.zeros((3,2,2))
        for i in range(0,3):
            c[i] = np.array([[np.cos(thetas[i]),-np.sin(thetas[i])],[np.sin(thetas[i]),np.cos(thetas[i])]])
        
        beta = np.sin(thetas[0]-thetas[2]) / np.sin(thetas[2]-thetas[1])    
        gamma = np.sin(thetas[1]-thetas[0]) / np.sin(thetas[2]-thetas[1])    
        T0 = np.matmul(c[0].T,G_p_Lis[0])
        T1 = beta*(np.matmul(c[1].T,G_p_Lis[1]))
        T2 = gamma*(np.matmul(c[2].T,G_p_Lis[2]))  
        u = T0+T1+T2
        u_cap = -u/np.linalg.norm(u)
        
        #Using Eq 5.50, calculate phi
        phi = np.arccos(u_cap[0])  

        #Using Eq 5.53, calculate dj
        v = np.zeros((3,2))
        for i in range(0,3):
            v[i] = -np.sin(thetas[i]+phi),np.cos(thetas[i]+phi).T
        d0 = ((1/np.sin(thetas[1]-thetas[0]))*v[1]).T@(G_p_Lis[1] - G_p_Lis[0])
        d1 = ((1/np.sin(thetas[2]-thetas[1]))*v[2]).T@(G_p_Lis[2] - G_p_Lis[1])
        d2 = ((1/np.sin(thetas[1]-thetas[2]))*v[1]).T@(G_p_Lis[1] - G_p_Lis[2])
        c_the_phi = np.zeros((3,2,2))
        for i in range(0,3):
            c_the_phi[i] = np.array([[np.cos(thetas[i]+phi),-np.sin(thetas[i]+phi)],[np.sin(thetas[i]+phi),np.cos(thetas[i]+phi)]])
        
        e = [1,0]
        # position = G_p_Lis[0] - d0*(c_the_phi[0]@e)
        position = G_p_Lis[2] - d2*(c_the_phi[2]@e)
        pose = [position[0],position[1],phi]
        return pose
        
        
# Task 2:
def nonLinearLSRange(dists,sigmas,G_p_Lis,G_p_R_0,alpha,n_iter): 
    # TODO: Use Gauss-Newton to iteratively solve the Weighted Non-Linear Least Squares Problem
    # Return a 2*1 nparray
    R = np.zeros((10,2,2))
    for i in range(0,G_p_Lis.shape[0]):
        R[i] = np.eye(2)*(sigmas[i]**2)
    Rinv = np.linalg.inv(R)
    G_p_R_curr = np.array(G_p_R_0)
    for i in range(n_iter):
        t2, t3 = 0,0
        hx = np.linalg.norm(G_p_Lis-G_p_R_curr,axis=1)[:,np.newaxis]
        hx_t = np.concatenate((hx,hx),axis=1)
        H = ((G_p_R_curr - G_p_Lis)/hx)[:,np.newaxis]
        t1 = dists - hx     
        for i in range(len(G_p_Lis)):
            t2 += (H[i]@np.linalg.inv(R[i])@H[i].T)
            t3 += (H[i]@np.linalg.inv(R[i])*(dists[i]-hx[i])).T
        weight = (1/t2)*t3
        w=np.squeeze(weight.T)
        G_p_R_curr+=w
    return(G_p_R_curr)

class Graph: #(DFS Algorithm for Task 3 to find cluster centers) 
 
    def __init__(self, row, col, g):
        self.ROW = row
        self.COL = col
        self.graph = g
 
    def isSafe(self, i, j, visited):

        return (i >= 0 and i < self.ROW and
                j >= 0 and j < self.COL and
                not visited[i][j] and self.graph[i][j])
      
    def DFS(self, i, j, visited):

        rowNbr = [-1, -1, -1,  0, 0,  1, 1, 1];
        colNbr = [-1,  0,  1, -1, 1, -1, 0, 1];
         
        # Mark this cell as visited
        visited[i][j] = True

        for k in range(8):
            if self.isSafe(i + rowNbr[k], j + colNbr[k], visited):
                self.DFS(i + rowNbr[k], j + colNbr[k], visited)
 
 

    def countIslands(self):

        visited = [[False for j in range(self.COL)]for i in range(self.ROW)]

        count = 0
        islands = []
        for i in range(self.ROW):
            for j in range(self.COL):
                # If a cell with value 1 is not visited yet,
                # then new island found
                if visited[i][j] == False and self.graph[i][j] > 0:
                    # Visit all cells in this island
                    # and increment island count
                    self.DFS(i, j, visited)
                    count += 1
                    islands.append([i, j])
 
        return np.array(islands)

          
# Task 3:
def findLines(dists): # a vector of distances to the nearest ojbect along a 180 degree sweep
    # TODO: Find all lines in the lidar data using Hough transform. 
    # Return a 2*K matrix where each column corresponds to d and θ of each line.
    rho_min = min(dists)
    rho_max = max(dists)
    theta_ground = np.linspace(0,np.radians(180),len(dists))
    theta = np.linspace(0,np.radians(180),36) #shape : (36,)
    rhos = np.linspace(rho_min,rho_max,10) #shape : (10,)
    acc = np.zeros((theta.shape[0],rhos.shape[0])) #Shape:(36,10)
    dist_thresh = 0.05
    
    for angle in range(len(theta)): #runs 36 times
        for rho in range(len(rhos)):  #runs 10 times
            dj =rhos[rho]
            thetaj=theta[angle]
            counter =0
            minimum=1
            index=0
            for d in range(len(dists)): #runs 512 times
                di = dists[d]
                ti = theta_ground[d]
                test = np.abs(dj - di*np.cos(thetaj-ti))
                # if test< minimum and test<dist_thresh:
                if test<dist_thresh:
                    counter +=1
             
            acc[angle,rho] = counter
               
    acc[acc<100]=0
    visit = np.zeros(acc.shape)
    
    row = len(acc)
    col = len(acc[0])
     
    g = Graph(row, col, acc)
     
    print ("Number of islands is:")
    indices = g.countIslands()
    for i in range(len(indices)):
        p, q = indices[i]
        print(np.rad2deg(theta[p]), rhos[q])
    

if __name__ == "__main__":
    
    robot = Supervisor()
    
    camera = robot.getDevice('camera')
    camera.enable(1)
    
    lidar = robot.getDevice('lidar')
    lidar.enable(1)
    lidar.enablePointCloud()
    
    if camera.hasRecognition():
        camera.recognitionEnable(1)
        camera.enableRecognitionSegmentation()
    else:
        print("Your camera does not have recognition")
    
    timestep = int(robot.getBasicTimeStep())
    
    while robot.step(timestep) != -1:
        # Task 1: Bearing measurments estimation
        recObjs = camera.getRecognitionObjects()
        recObjsNum = camera.getRecognitionNumberOfObjects()
        thetas = np.zeros(recObjsNum)
        G_p_Lis = []
        for i in range(0, recObjsNum):
            thetas[i] = posOfImgToBearing(recObjs[i].get_position_on_image()[0], w, fov)
            objPos = robot.getFromId(recObjs[i].get_id()).getPosition()
            G_p_Lis.append([objPos[0], objPos[1]])
        # print(G_p_Lis)
        G_p_R_Bearing = threeLandmarksBearing(thetas, G_p_Lis)
        
        print("Task 1:\nEstimation:\n")
        print(G_p_R_Bearing)
        print("\n")
            
        # Task 2: Range measurments estimation
        G_p_R = [0,0]
        # Task 2 bonus point: calculate the initial guess
        G_p_R_0 = np.random.random_sample(2)
        n_pts = 10
        G_p_Li = np.zeros((n_pts, 2), dtype=np.float64)
        dists = np.zeros(n_pts,  dtype=np.float64)
        sigmas = np.full(n_pts, 0.2, dtype=np.float64)
        noise = np.random.normal(0.0, sigmas, n_pts)
        
        for i in range(0, n_pts):
            G_p_Li[i] = 2.0 * np.random.random_sample(2)
            dists[i] = np.linalg.norm((G_p_Li[i] - G_p_R)) + noise[i]
        
        G_p_R_Range = nonLinearLSRange(dists, sigmas, G_p_Li, G_p_R_0, 0.01, 100)
        
        print("Task 2:\nInitial guess:\n")
        print(G_p_R_0)
        print("Estimation:\n")
        print(G_p_R_Range)
        
        # # Task 3: Line segmentation extraction
        lines = findLines(np.array(lidar.getRangeImage()))
        print("Task 3:\nLines:\n")
        print(lines)
        
        
        break
