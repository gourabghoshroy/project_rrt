# -*- coding: utf-8 -*-
"""
Spyder Editor
Autonomous Robots Project
RRT algorithm
"""

import numpy as np
import math
import matplotlib.pyplot as plt
from skimage import io


def rrt(map, q_start, q_goal, num, delta_q, p):
    plt.figure(1)
    plt.imshow(map, cmap='Greys')
    
    m=map.shape[0]
    n=map.shape[1]
    edges=np.ndarray([0,2])
    vertices=np.ndarray([1,2])
    vertices[0,:]=q_start

    i=0;
    while(i<num):
        while(1):
            new=np.random.uniform(0,1)
            if new>p:
                x=int((np.random.uniform(0,1))*m-1)
                y=int((np.random.uniform(0,1))*n-1)
                if x>0 and y>0:
                    if map[x,y]==0:
                        break
            else:
                y=q_goal[0]
                x=q_goal[1]
                break
            
        o=vertices.shape[0]
        dist=m*n
        for j in range(o):
            distn=np.sqrt((vertices[j,0]-y)*(vertices[j,0]-y) + (vertices[j,1]-x)*(vertices[j,1]-x))
            if distn<dist:
                dist=distn
                q_near=j
        if dist>delta_q:
            dist=delta_q
        
        gap=3
        check=0
        msz=int(dist/gap)
        theta=math.atan2(x-vertices[q_near,1],y-vertices[q_near,0])

        for k in range(msz):
            x1=int(k*gap*math.sin(theta)+vertices[q_near,1])
            y1=int(k*gap*math.cos(theta)+vertices[q_near,0])
            if map[x1,y1]==1: 
                check=1
                break
        x=int(dist*math.sin(theta)+vertices[q_near,1]);
        y=int(dist*math.cos(theta)+vertices[q_near,0]);
        if check==1 or map[abs(x),abs(y)]==1:
            continue
        
        vertices=np.vstack((vertices,[y, x]))
        edges=np.vstack((edges,[q_near, i+1]))

        if x==q_goal[1] and y==q_goal[0]:
            break
        i+=1
        print i
        
    if i==1000:
        print('The path could not be found');
        return
    
    plt.scatter(q_goal[0], q_goal[1])
    plt.scatter(q_start[0], q_start[1])

    for i in range(edges.shape[0]):
        plt.plot(np.array([vertices[edges[i,0],0], vertices[edges[i,1],0]]), np.array([vertices[edges[i,0],1], vertices[edges[i,1],1]]),c='red')

    k=0
    path=np.array([])
    df=edges[edges.shape[0]-1,1]
    path=np.hstack((path,df))
    while(1):
        if path[k]==0:
            break
        for j in range(edges.shape[0]):
            if (edges[j,1]==path[k]):
                k=k+1
                path=np.hstack((path,edges[j,0]))
                break

    drawp=np.ndarray([0,2])
    for i in range(path.shape[0]-1):
        drawp=np.vstack((drawp,[path[i], path[i+1]]))

    for i in range(drawp.shape[0]):
        plt.plot(np.array([vertices[drawp[i,0],0], vertices[drawp[i,1],0]]), np.array([vertices[drawp[i,0],1], vertices[drawp[i,1],1]]),c='green')


    path=np.sort(path)

    i=0;
    path_smooth=np.array([i])
    while(1):
        for j in range(path.shape[0]-1, i-1, -1):
            check=0
            a=vertices[path_smooth[path_smooth.shape[0]-1],0]
            b=vertices[path[j],0]
            c=vertices[path_smooth[path_smooth.shape[0]-1],1]
            d=vertices[path[j],1]
            r=math.sqrt(math.pow(a-b,2)+math.pow(c-d,2))
            theta=math.atan2((d-c),(b-a))

            gap=2
            msz=int(r/gap)

            for k in range(msz):
                x1=int(k*gap*math.sin(theta)+c)
                y1=int(k*gap*math.cos(theta)+a)
                if map[x1,y1]==1: 
                    check=1
                    break
            if j==i or (check==0):
                path_smooth=np.hstack((path_smooth, path[j]))
                i=j
                break
        if (path_smooth[path_smooth.shape[0]-1]==path[path.shape[0]-1]):
            break


    drawp=np.ndarray([0,2])
    for i in range(path_smooth.shape[0]-1):
        drawp=np.vstack((drawp,[path_smooth[i], path_smooth[i+1]]))

    for i in range(drawp.shape[0]):
        plt.plot(np.array([vertices[drawp[i,0],0], vertices[drawp[i,1],0]]), np.array([vertices[drawp[i,0],1], vertices[drawp[i,1],1]]),c='blue')


    waypts=np.ndarray([0,2])
    for i in range(path_smooth.shape[0]):
        waypts=np.vstack((waypts,vertices[path_smooth[i],:]))

    return waypts


map = io.imread('img3.jpg',1)
map=1-map

q_start=np.array([50, 50])
q_goal=np.array([450, 350])
k=1000
delta_q=15
p=0.5
delta=1


pts=rrt(map,q_start,q_goal,k,delta_q,p)
print pts
