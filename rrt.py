import sys
import random
import matplotlib.pyplot as plt
import matplotlib.path as mpPath
import matplotlib.patches as patches
import numpy as np
import matplotlib.lines as mlines
import math

'''
Set up matplotlib to create a plot with an empty square
'''
def setupPlot():
    fig = plt.figure(num=None, figsize=(5, 5), dpi=120, facecolor='w', edgecolor='k')
    ax = fig.subplots()
    ax.set_axisbelow(True)
    ax.set_ylim(-1, 11)
    ax.set_xlim(-1, 11)
    ax.grid(which='minor', linestyle=':', alpha=0.2)
    ax.grid(which='major', linestyle=':', alpha=0.5)
    return fig, ax

'''
Make a patch for a single pology 
'''
def createPolygonPatch(polygon, color):
    verts = []
    codes= []
    for v in range(0, len(polygon)):
        xy = polygon[v]
        verts.append((xy[0], xy[1]))
        if v == 0:
            codes.append(mpPath.Path.MOVETO)
        else:
            codes.append(mpPath.Path.LINETO)
    verts.append(verts[0])
    codes.append(mpPath.Path.CLOSEPOLY)
    path = mpPath.Path(verts, codes)
    patch = patches.PathPatch(path, facecolor=color, lw=1)

    return patch
    

'''
Render the problem  
'''
def drawProblem(robotStart, robotGoal, polygons):
    _, ax = setupPlot()
    patch = createPolygonPatch(robotStart, 'green')
    ax.add_patch(patch)    
    patch = createPolygonPatch(robotGoal, 'red')
    ax.add_patch(patch)    
    for p in range(0, len(polygons)):
        patch = createPolygonPatch(polygons[p], 'gray')
        ax.add_patch(patch)    
    plt.show()

'''
Display the RRT and Path
'''
def displayRRTandPath(points, adjListMap, path, robotStart=None, robotGoal=None, polygons=None):
    _, ax = setupPlot()
    if robotStart != None and robotGoal != None and polygons != None:
        patch = createPolygonPatch(robotStart, 'green')
        ax.add_patch(patch)    
        patch = createPolygonPatch(robotGoal, 'red')
        ax.add_patch(patch)    
        for p in range(0, len(polygons)):
            #print("adding patch")
            patch = createPolygonPatch(polygons[p], 'gray')
            ax.add_patch(patch)    
    
    drawLines(adjListMap, points, _, ax)
    drawpath(path, points, _, ax)
    ax.plot()
    plt.show()
    return

def drawLines(adjMap, vertices, fig, ax):
    #print("vertices: " + str(vertices))
    for v in vertices:
        #print("adjMap["+str(v)+"]: " + str(adjMap[v]))
        for label in adjMap[v]:
            #print("line: " + str(v) + " , " + str(label))
            newline(vertices[v], vertices[label], 'black')

def newline(p1, p2, c):
    ax = plt.gca()
    line = [(p1[0], p1[1]), (p2[0], p2[1])]
    (linxs, linys) = zip(*line)
    ax.add_line(mlines.Line2D(linxs, linys, linewidth=1, color=c))

def drawpath(path, vertices, fig, ax):
    if path == [] or path == None:
        return
    curr = path[0]
    for i in range(0,len(path)):
        if i != len(path)-1:
            #print("path line: " + str(vertices[path[i]])+", "+str(vertices[path[i+1]]))
            newline(vertices[path[i]], vertices[path[i+1]], 'orange')


'''
Grow a simple RRT 
'''
def growSimpleRRT(points):
    newPoints = {}
    adjListMap = {}
    
    pointCount = 1

    for key in points:
        newP = points[key]
        if key == 1:
            newPoints[1] = points[1]
            adjListMap[1] = []
        else:
            minDist = float('inf')
            closestPoint = [] ## [x, y]
            closestEdge = [] ## [p, q]
            for sourceNodeKey in adjListMap:
                p = newPoints[sourceNodeKey]
                for destNodeKey in adjListMap[sourceNodeKey]:
                    q = newPoints[destNodeKey]
                    (qN, dist) = findNearestPointOnEdge(p, q, newP)
                    if dist < minDist:
                        minDist = dist
                        closestPoint = qN
                        closestEdge = [p, q]
            if minDist == float('inf'):
                pointCount = pointCount + 1 
                newPoints[key] =  points[key]
                adjListMap[key] = [1]
                adjListMap[1] = [key]
            else:
                if closestEdge != [] and (closestPoint == closestEdge[0] or closestPoint == closestEdge[1]):
                    pointCount = pointCount + 1
                    newPoints[pointCount] = newP
                    adjListMap[pointCount] = [getPointKey(closestPoint, newPoints)]
                else:#new pt on the edge
                    pointCount = pointCount + 1
                    newPoints[pointCount] = newP
                    adjListMap[pointCount] = [pointCount+1]
                    pointCount = pointCount + 1
                    newPoints[pointCount] = closestPoint
                    adjListMap[pointCount] = [pointCount - 1] #adj to the new point
                    pKey = getPointKey(closestEdge[0], newPoints)
                    qKey = getPointKey(closestEdge[1], newPoints)
                    adjListMap[pointCount].append(pKey)
                    adjListMap[pointCount].append(qKey)
                    adjListMap[pKey].append(pointCount)
                    adjListMap[qKey].append(pointCount)
                    try:
                        adjListMap[pKey].remove(qKey)
                    except Exception as e:
                        raise e
                    try:
                        adjListMap[qKey].remove(pKey)
                    except Exception as e:
                        print("")
    return newPoints, adjListMap
'''
Grow a simple RRT with edge detection with obstacles
'''
def growSimpleRRTwithObstacles(points, obstacles):
    newPoints = {}
    adjListMap = {}
    unaddedPoints = []
    pointCount = 1

    for k in points:
        nextPoint = points[k]

        if k == 1:#root
            newPoints[1] = points[1]
            adjListMap[1] = []
        else:#k>1... check for every edge in tree find the closest one
            minDistance = float('inf')
            closestPoint = []
            edgeUsed = [[],[]]
            for sk in adjListMap:
                for dk in adjListMap[sk]:
                    p = newPoints[sk]
                    q = newPoints[dk]
                    (nearestPointOnTree, distance) = findNearestPointOnEdge(p, q, nextPoint)
                    
                    if distance < minDistance:
                        ##check collision
                        newEdge = [nextPoint, nearestPointOnTree]
                        if isCollisionFree(newEdge, (0,0), obstacles):
                            minDistance = distance
                            closestPoint = nearestPointOnTree
                            edgeUsed = [p,q]

            if pointCount == 1:
                newEdge = [nextPoint, newPoints[1]]
                if isCollisionFree(newEdge, (0,0), obstacles):
                    minDistance = squaredDist(nextPoint, newPoints[1])
                    closestPoint = newPoints[1]
                    edgeUsed = [newPoints[1],newPoints[1]]

            if minDistance != float('inf'):
                if closestPoint == edgeUsed[0] or closestPoint == edgeUsed[1]:
                    pointCount = pointCount + 1
                    newPoints[pointCount] = nextPoint
                    adjListMap[pointCount] = [getPointKey(closestPoint, newPoints)]
                else:#new pt on the edge
                    pointCount = pointCount + 1
                    newPoints[pointCount] = nextPoint
                    adjListMap[pointCount] = [pointCount+1]
                    pointCount = pointCount + 1
                    newPoints[pointCount] = closestPoint
                    adjListMap[pointCount] = [pointCount - 1] #adj to the new point
                    pKey = getPointKey(edgeUsed[0], newPoints)
                    qKey = getPointKey(edgeUsed[1], newPoints)
                    adjListMap[pointCount].append(pKey)
                    adjListMap[pointCount].append(qKey)
                    adjListMap[pKey].append(pointCount)
                    adjListMap[qKey].append(pointCount)
                    try:
                        adjListMap[pKey].remove(qKey)
                    except Exception as e:
                        raise e
                    try:
                        adjListMap[qKey].remove(pKey)
                    except Exception as e:
                        print("")
            else:
                unaddedPoints.append(nextPoint)



    print("Added Pts: " + str(newPoints))
    print("\nUnaddedPts: " + str(unaddedPoints))
    
    retryCount = 10
    while unaddedPoints and retryCount > 0:
        pt = random.choice(unaddedPoints)
        (newPoints1, adjListMap1) = addPointToTree(pt,newPoints, adjListMap,obstacles)
        newPoints = newPoints1
        adjListMap = adjListMap1
        if pt in newPoints:
            print("Removing pt : " + str(pt))
            unaddedPoints.remove(pt)
        retryCount = retryCount - 1

    return newPoints, adjListMap
'''
expand tree function to add point to the tree if possible
'''
def addPointToTree(point, newPoints, adjListMap, obstacles):
    print("Adding pt: " + str(point))
    pointCount = len(adjListMap)
    unaddedPoints = []

    minDistance = float('inf')
    closestPoint = []
    edgeUsed = [[],[]]
    for sk in adjListMap:
        for dk in adjListMap[sk]:
            p = newPoints[sk]
            q = newPoints[dk]
            (nearestPointOnTree, distance) = findNearestPointOnEdge(p, q, point)
            
            if distance < minDistance:
                ##check collision
                newEdge = [point, nearestPointOnTree]
                if isCollisionFree(newEdge, (0,0), obstacles):
                    minDistance = distance
                    closestPoint = nearestPointOnTree
                    edgeUsed = [p,q]
    if minDistance != float('inf'):
        if closestPoint == edgeUsed[0] or closestPoint == edgeUsed[1]:
            pointCount = pointCount + 1
            newPoints[pointCount] = point
            adjListMap[pointCount] = [getPointKey(closestPoint, newPoints)]
        else:#new pt on the edge
            pointCount = pointCount + 1
            newPoints[pointCount] = point
            adjListMap[pointCount] = [pointCount+1]
            pointCount = pointCount + 1
            newPoints[pointCount] = closestPoint
            adjListMap[pointCount] = [pointCount - 1] #adj to the new point
            pKey = getPointKey(edgeUsed[0], newPoints)
            qKey = getPointKey(edgeUsed[1], newPoints)
            adjListMap[pointCount].append(pKey)
            adjListMap[pointCount].append(qKey)
            adjListMap[pKey].append(pointCount)
            adjListMap[qKey].append(pointCount)
            try:
                adjListMap[pKey].remove(qKey)
            except Exception as e:
                raise e
            try:
                adjListMap[qKey].remove(pKey)
            except Exception as e:
                print("")
    return (newPoints, adjListMap)

'''
returns the key value of a given point
'''
def getPointKey(p, points):
    for i in points:
        if p == points[i]:
            return i
    return -1
'''
Calc square distance between two points
'''
def squaredDist(p, q):
    return (p[0] - q[0])**2 + (p[1] - q[1])**2

'''
Calc Nearest Point On Edge
'''
def findNearestPointOnEdge(p, q, q_rand):
    [x1, y1] = p
    [x2, y2] = q
    [xr, yr] = q_rand

    m1 = (y2 - y1)/(x2-x1)
    m2 = -1/m1

    b1 = y1 - m1*x1
    b2 = yr - m2*xr

    x_nearest = (b2 - b1)/(m1 - m2)
    y_nearest = m1*x_nearest + b1

    if (x_nearest < max(x1, x2) and x_nearest > min(x1, x2)) and (y_nearest < max(y1, y2) and y_nearest > min(y1, y2)):
        return ([x_nearest, y_nearest], squaredDist(q_rand, [x_nearest, y_nearest]))
    if squaredDist(p, q_rand) < squaredDist(q, q_rand):
        return (p, squaredDist(p, q_rand))

    return (q, squaredDist(q, q_rand))


'''
Perform basic search 
'''
def basicSearch(tree, start, goal):
    if len(tree) <= 1:
        return []
    visited = [0]*len(tree)
    visited[1] = 1
    return dfs(tree, start, goal, visited)

'''
recursive dfs helper
'''
def dfs(tree, start, goal, visited):
    if start == goal:
        return [goal]
    if tree[start] == [] or tree[start] == None:
        return []
    for k in [i for i in tree[start] if visited[i] == 0]:
        visited[k] = 1
        path = dfs(tree, k, goal, visited)
        if path != []:
            path.insert(0,start)
            return path
    return []

'''
Collision checking
'''
# def isCollisionFree(robot, point, obstacles):
#     print("robot: " + str(robot))
#     print("point: " + str(point))
#     print("obstacles: " + str(obstacles))
#     for x in range(0,len(robot)):
#         X1 = robot[x][0]+point[0]
#         Y1 = robot[x][1]+point[1]
#         if(x == len(robot)-1):
#             X2 = robot[0][0]+point[0]
#             Y2 = robot[0][1]+point[1]
#         else:
#             X2 = robot[x+1][0]+point[0]
#             Y2 = robot[x+1][1]+point[1]

#         for obs in obstacles:
#             for y in range(0,len(obs)):
#                 X3 = obs[y][0]
#                 Y3 = obs[y][1]
#                 if (y == len(obs) - 1):
#                     X4 = obs[0][0]
#                     Y4 = obs[0][1]
#                 else:
#                     X4 = obs[y + 1][0]
#                     Y4 = obs[y + 1][1]
#                 if (intersectLines([X1,Y1], [X2,Y2], [X3, Y3], [X4, Y4]) == True):
#                     return False
#     return True

'''
Collision checking
'''
def isCollisionFree(robot, point, obstacles):
    print("robot: " + str(robot))
    print("point: " + str(point))
    print("obstacles: " + str(obstacles))
    for x in range(0,len(robot)):
        X1 = robot[x][0]+point[0]
        Y1 = robot[x][1]+point[1]
        if(x == len(robot)-1):
            X2 = robot[0][0]+point[0]
            Y2 = robot[0][1]+point[1]
        else:
            X2 = robot[x+1][0]+point[0]
            Y2 = robot[x+1][1]+point[1]
        # print("X1, Y1 = ", X1, ", ", Y1)
        # print("X2, Y2 = ", X2, ", ", Y2,"\n")
        for obs in obstacles:
            for y in range(0,len(obs)):
                X3 = obs[y][0]
                Y3 = obs[y][1]
                if (y == len(obs) - 1):
                    X4 = obs[0][0]
                    Y4 = obs[0][1]
                else:
                    X4 = obs[y + 1][0]
                    Y4 = obs[y + 1][1]

                # print("X3, Y3 = ", X3, ", ", Y3)
                # print("X4, Y4 = ", X4, ", ", Y4)
                if intersectLines([X1,Y1], [X2,Y2], [X3, Y3], [X4, Y4]):
                    # print("Intersection found")
                    return False
    return True

def intersectLines( pt1, pt2, ptA, ptB ): 
    DET_TOLERANCE = 0.00001
    x1, y1 = pt1;   x2, y2 = pt2
    dx1 = x2 - x1;  dy1 = y2 - y1
    x, y = ptA;   xB, yB = ptB;
    dx = xB - x;  dy = yB - y;
    # print("Stuff")
    DET = (-dx1 * dy + dy1 * dx)

    if math.fabs(DET) < DET_TOLERANCE: 
    	return False
    DETinv = 1.0/DET
    r = DETinv * (-dy  * (x-x1) +  dx * (y-y1))
    s = DETinv * (-dy1 * (x-x1) + dx1 * (y-y1))
    xi = (x1 + r*dx1 + x + s*dx)/2.0
    yi = (y1 + r*dy1 + y + s*dy)/2.0
    # print("Intersection: ",xi,",",yi)
    if xi <= max(x1, x2) and xi >= min(x1, x2) and yi <= max(y1, y2) and yi >= min(y1, y2):
        if xi <= max(x, xB) and xi >= min(x, xB) and yi <= max(y, yB) and yi >= min(y, yB):
            print("Intersection at: ", xi, ", ", yi)
            return True
    return False

'''
The full RRT algorithm
'''
def RRT(robot, obstacles, startPoint, goalPoint):
    points = dict()
    points2 = dict()
    tree = dict()
    path = []
    points[1] = startPoint
    points[2] = goalPoint
    modobstacles = obstacles[0:len(obstacles) - 2]  # so as to not include the surrounding area in point calc
    for init in range(0, 50):
        x = np.random.ranf() * 10  # we are given that the area is from 0 to 10 in a box
        y = np.random.ranf() * 10
        for obstacle in modobstacles:
            tempArray = np.empty((0, 2), float)
            for obs in range(0, len(obstacle)):
                tempArray = np.append(tempArray, np.array([obstacle[obs]]), axis=0)
            tempArray = np.append(tempArray, np.array([obstacle[0]]), axis=0)
            tempPath = mpPath.Path(tempArray)
            if (tempPath.contains_point((x, y))):
                break
            else:
                pass
        if (tempPath.contains_point((x, y)) == False):
            points[len(points) + 1] = (x, y)
    # goal = len(points)+1
    # points[goal] = goalPoint
    points2, tree = growSimpleRRTwithObstacles(points, obstacles)

    if dictsearch(goalPoint,points2) == 0:
        path = []
    else:
        path = basicSearch(tree, 1, dictsearch(goalPoint, points2))
    if dictsearch(goalPoint,points) == 0:
        points[len(points) + 1] = goalPoint
    while path == [] and len(points )<100:
        for init in range(0, 10):
            x = np.random.ranf() * 10  # we are given that the area is from 0 to 10 in a box
            y = np.random.ranf() * 10
            for obstacle in modobstacles:
                tempArray = np.empty((0, 2), float)
                for obs in range(0, len(obstacle)):
                    tempArray = np.append(tempArray, np.array([obstacle[obs]]), axis=0)
                tempArray = np.append(tempArray, np.array([obstacle[0]]), axis=0)
                tempPath = mpPath.Path(tempArray)
                if (tempPath.contains_point((x, y))):
                    break
                else:
                    pass
            if (tempPath.contains_point((x, y)) == False):
                points[len(points) + 1] = (x, y)
        if dictsearch(goalPoint, points)==False:
            points[len(points) + 1] = (x, y)
        points2, tree = growSimpleRRTwithObstacles(points, obstacles)
        if dictsearch(goalPoint, points2)==0:
            path = []

        else:
            pass
            path = basicSearch(tree, 1, 2)
        points[len(points) + 1] = goalPoint
    points2, tree = growSimpleRRTwithObstacles(points, obstacles)
    path = basicSearch(tree, 1, 2)

    return points2, tree, path

def dictsearch(val, dict):
    for items in dict:
        if val ==dict[items]:
            return items
    return 0

    return points, tree, path
def ccw(A,B,C):
    return ((C[1]-A[1])) * (B[0]-A[0]) > ((B[1]-A[1]) * (C[0]-A[0]))

def intersect(A, B, C, D):
    return ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D)


def main(filename, x1, y1, x2, y2, display=''):
    # Read data and parse polygons
    lines = [line.rstrip('\n') for line in open(filename)]
    robot = []
    obstacles = []
    for line in range(0, len(lines)):
        xys = lines[line].split(';')
        polygon = []
        for p in range(0, len(xys)):
            xy = xys[p].split(',')
            polygon.append((float(xy[0]), float(xy[1])))
        if line == 0 :
            robot = polygon
        else:
            obstacles.append(polygon)

    # Print out the data
    print("Robot:")
    print(str(robot))
    print("Pologonal obstacles:")
    for p in range(0, len(obstacles)):
        print(str(obstacles[p]))
    print("")

    # Visualize
    if display == 'display1':
        robotStart = [(x + x1, y + y1) for x, y in robot]
        robotGoal = [(x + x2, y + y2) for x, y in robot]
        drawProblem(robotStart, robotGoal, obstacles)

    # Example points for calling growSimpleRRT
    # You should expect many mroe points, e.g., 200-500
    points = dict()
    points[1] = (5, 5)
    points[2] = (7, 8.2)
    points[3] = (6.5, 5.2)
    points[4] = (0.3, 4)
    points[5] = (6, 3.7)
    points[6] = (9.7, 6.4)
    points[7] = (4.4, 2.8)
    points[8] = (9.1, 3.1)
    points[9] = (8.1, 6.5)
    points[10] = (0.7, 5.4)
    points[11] = (5.1, 3.9)
    points[12] = (2, 6)
    points[13] = (0.5, 6.7)
    points[14] = (8.3, 2.1)
    points[15] = (7.7, 6.3)
    points[16] = (7.9, 5)
    points[17] = (4.8, 6.1)
    points[18] = (3.2, 9.3)
    points[19] = (7.3, 5.8)
    points[20] = (9, 0.6)

    # Printing the points
    print("")
    print("The input points are:")
    print(str(points))
    print("")
    points, adjListMap = growSimpleRRT(points)
    print("")
    print("The new points are:")
    print(str(points))
    print("")
    print("")
    print("The tree is:")
    print(str(adjListMap))
    print("")

    # Search for a solution  
    # change 1 and 20 as you want
    path = basicSearch(adjListMap, 1, 2)
    print("")
    print("The path is:")
    print(str(path))
    print("")

    # Your visualization code 
    if display == 'display2':
        displayRRTandPath(points, adjListMap, path) 

    # Solve a real RRT problem
    points, adjListMap, path = RRT(robot, obstacles, (x1, y1), (x2, y2))
    
    # Your visualization code 
    if display == 'display3':
        robotStart = [(x + x1, y + y1) for x, y in robot]
        robotGoal = [(x + x2, y + y2) for x, y in robot]
        isCollisionFree(robot, [0,0], obstacles)
        displayRRTandPath(points, adjListMap, path, robotStart, robotGoal, obstacles) 


if __name__ == "__main__":
    # Retrive file name for input data
    if(len(sys.argv) < 6):
        print("Five arguments required: python spr.py [env-file] [x1] [y1] [x2] [y2]")
        exit()
    
    filename = sys.argv[1]
    x1 = float(sys.argv[2])
    y1 = float(sys.argv[3])
    x2 = float(sys.argv[4])
    y2 = float(sys.argv[5])
    display = ''
    if(len(sys.argv) == 7):
        display = sys.argv[6]

    main(filename, x1, y1, x2, y2, display)
