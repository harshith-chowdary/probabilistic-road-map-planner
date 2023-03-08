import math
import shapely.geometry
#from shapely.geometry.polygon import polygon
#from shapely.geometry import Point
import matplotlib.pyplot as plt
from matplotlib import path
from matplotlib.path import Path
import random
from scipy.spatial.distance import cdist
from sklearn.neighbors import NearestNeighbors

obs1=[[5,11],[14,5],[11,20]]
obs2=[[12,29],[15,31],[15,34],[12,34]]
obs3=[[25,9],[25,20],[32,15],[35,6]]
obs4=[[53,32],[39,32],[36,27],[42,19],[53,24]]

poly1=shapely.geometry.Polygon(obs1)
poly2=shapely.geometry.Polygon(obs2)
poly3=shapely.geometry.Polygon(obs3)
poly4=shapely.geometry.Polygon(obs4)

#to draw obstacles
def draw_poly(poly):
    polygon=shapely.geometry.Polygon(poly)   
    '''plt.rcParams["figure.figsize"] = [7.00, 3.50]
    plt.rcParams["figure.autolayout"] = True'''
    x, y = polygon.exterior.xy
    plt.plot(x, y, c="black")

def is_in_poly(point):
    p=path.Path(obs1)
    if p.contains_point(point):    
        return True
    p=path.Path(obs2)
    if p.contains_point(point):    
        return True
    p=path.Path(obs3)
    if p.contains_point(point):    
        return True
    p=path.Path(obs4)
    if p.contains_point(point):    
        return True
    return False

def nodes_plotter(n):
    len=0
    nodes=[]
    rx=[]
    ry=[]
    gx=[]
    gy=[]
    while len<n:
        x=random.randint(0,60)
        y=random.randint(0,40)
        xd=random.randint(0,9)/10
        yd=random.randint(0,9)/10
        node=[x+xd,y+yd]
        #if node not in nodes:
        '''if is_in_poly(node):
            plt.scatter(node[0],node[1],marker='o',mfc="red")
        else:
            plt.scatter(node[0],node[1],marker='o',mfc="green")
            len+=1'''
        if is_in_poly(node):
            rx.append(node[0])
            ry.append(node[1])
        else:
            gx.append(node[0])
            gy.append(node[1])
            len+=1
            nodes.append(node)

# un comment this part to make nodes visible on plot :) :) :) :) :) :) :) :) :) :) :) :) :) :) :) :) :) :) :) :) :) :) :) :)
        plt.scatter(rx, ry, color="red", s=7, alpha=0.5)
        plt.scatter(gx, gy, color="green", s=7, alpha=0.5)

    return nodes

'''

def closest_nodes(k,node,nodes):
    cnodes=[]
    nodes.remove(node)
    while len(cnodes)<k:
        cnode=nodes[cdist([node], nodes).argmin()]
        cnodes.append(cnode)
        nodes.remove(cnode)
    return cnodes

def closest_node(node,nodes):
    return nodes[cdist([node], nodes).argmin()]

'''

import numpy as np

def distance(pt_1, pt_2):
    pt_1 = np.array((pt_1[0], pt_1[1]))
    pt_2 = np.array((pt_2[0], pt_2[1]))
    return np.linalg.norm(pt_1-pt_2)

def closest_node(node, nodes):
    pt = []
    dist = 9999999
    for n in nodes:
        if distance(node, n) <= dist:
            dist = distance(node, n)
            pt = n
    return pt

def closest_node_new(node, nodes):
    
    fx = node[0]-math.floor(node[0])
    fy = node[1]-math.floor(node[1])

    node[0] = math.floor(node[0])
    node[1] = math.floor(node[1])

    if fx>=0.5:
        node[0]+=1
    if fy>=0.5:
        node[1]+=1

    [x,y] = node
    if node in nodes:
        return node

    nnodes = [[x+1,y],[x,y+1],[x-1,y],[x,y-1]]
    for node in nnodes:
        if node in nodes:
            return node


def closest_nodes_me(node,k,nodes):

    '''
    try:
        nodes.remove(node)
    except ValueError:
        pass
    '''

    dict={}
    for nod in nodes:
        dic={tuple(nod) : math.dist(node, nod)}
        dict.update(dic)

    sorted_dic = sorted(dict.items(), key=lambda kv:kv[1])
    return sorted_dic[1:k+5]

    '''
    cnodes=[]
    d1=d2=d3=100
    for nod in nodes:
        dist=math.dist(nod,node)
        if dist<d1:
            d3=d2
            d2=d1
            d1=dist
            cnodes.insert(0,nod)
        elif dist<d2:
            d3=d2
            d2=dist
            cnodes.insert(1,nod)
        elif dist<d3:
            d3=dist
            cnodes.insert(2,nod)
    dists=[d1,d2,d3]
    return cnodes,dists
    '''

def valid_dir_link(p1,p2):
    line_seg=shapely.geometry.LineString([p1,p2])

    if line_seg.intersects(poly1):
        return False
    elif line_seg.intersects(poly2):
        return False
    elif line_seg.intersects(poly3):
        return False
    elif line_seg.intersects(poly4):
        return False
    return True

def direct_links(n,k,points):
    dic_links={}
    links=[]
    for i in range(0,n):
        try:
            node=points[i]
        except IndexError:
            break
        #print("Bruh : ",node)
        xn=node[0]
        yn=node[1]
        nps=closest_nodes_me(node,k,points)
        #print(nps)
        lst=[]
        j=0
        while len(lst)<k:
            try:
                np=nps[j][0]
            except IndexError:
                break
            if valid_dir_link(node,np)==True:
                #print("Yay")
                xnn=np[0]
                ynn=np[1]
                X=[xn,xnn]
                Y=[yn,ynn]
                #print("Path to ",xnn,ynn,"from ",xn,yn)
     # un comment below statement if every edge is required on plot :) :) :) :) :) :) :) :) :) :) :) :) :) :) :) :) :) :)
                #plt.plot(X,Y,'o-g')
                links.append([node,np])
                lst.append((np,nps[j][1]))
            j+=1
        dic={tuple(node):lst}
        dic_links.update(dic)
    return links,dic_links

draw_poly(obs1)
draw_poly(obs2)
draw_poly(obs3)
draw_poly(obs4)
#plt.show()

N=1000
K=10

nodes_list=nodes_plotter(N)

#print(nodes_list)
#plt.show()
links_list,linked_dic=direct_links(N,K,nodes_list)
#plt.show()
#print(linked_dic)

#CHECK POINT 1 CLEARED by now

q1=[(6,32),(17,8)]
q2=[(22,12),(39,18)]
q3=[(21,32),(54,12)]
queries=[q1, q2, q3]


''' Using A* Algorithm

import astar

adjacency_list=linked_dic
graph=astar.Graph(adjacency_list)

print("********************** PATH STARTS HERE **********************")

for q in queries:
    snode=tuple(closest_node(q[0],nodes_list))
    gnode=tuple(closest_node(q[1],nodes_list))
    proto_path=graph.a_star_algorithm(snode,gnode)
    print(proto_path)
    ppx,ppy=[],[]
    for node in proto_path:
        ppx.append(node[0])
        ppy.append(node[1])
    plt.plot(ppx,ppy,'o-o')

'''
import dijkstra as dj
graph = dj.Graph(nodes_list, linked_dic)

def collinear(n1, n2, n3):
    (x1,y1)=n1
    (x2,y2)=n2
    (x3,y3)=n3
    a = x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)
 
    if (a == 0):
        return True
    return False

def sampler(path):
    n=len(path)-1
    i=0
    j=1
    index = [1]*n
    while j<n:
        if collinear(path[i],path[j],path[j+1]):
            j+=1
        else:
            if valid_dir_link(path[i],path[j+1]):
                index[j]=0
                j+=1
            else:
                i=j
                #index[i]=1                               work here !!!
    xs = []
    ys = []
    for i in range(0,n):
        if index[i]:
            xs.append(path[i][0])
            ys.append(path[i][1])
    plt.plot(xs, ys, 'o:b')


# Using Dijstra's Algorithm
print()

for q in queries:
    snode=tuple(closest_node(list(q[0]), nodes_list))
    gnode=tuple(closest_node(list(q[1]), nodes_list))
    [x1,y1]=list(q[0])
    [xs,ys]=list(snode)
    #plt.plot([x1,xs],[y1,ys],'o-c')
    [x2,y2]=list(q[1])
    [xg,yg]=list(gnode)
    #plt.plot([x2,xg],[y2,yg],'o-c')
    print("****** QUERY ******")
    print("Nearest Node to START : ",snode)
    print("Nearest Node to GOAL  : ",gnode)
    print("______  END  ______")
    print()
    prev_nodes, proto_path = dj.dijkstra_algorithm(graph, snode)
    #print(proto_path)
    ppx,ppy=[gnode[0]],[gnode[1]]
    path_nodes = [gnode,gnode]
    while prev_nodes[gnode]!=snode:
        gnode=prev_nodes[gnode]
        path_nodes.insert(0,gnode)
        ppx.append(gnode[0])
        ppy.append(gnode[1])
    path_nodes.insert(0,snode)
    ppx.append(snode[0])
    ppy.append(snode[1])
    plt.plot(ppx,ppy,'o-y')
    plt.plot([x1,xs],[y1,ys],'o-c')
    plt.plot([x2,xg],[y2,yg],'o-c')
    #plt.show()
    sampler(path_nodes)

    '''
    ppx,ppy=[],[]
    for node in proto_path:
        ppx.append(node[0])
        ppy.append(node[1])
    plt.plot(ppx,ppy,'o-o')
    '''
plt.show()

#CHECK POINT 2 CLEARED by now