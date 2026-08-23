import math

def tsp_solver_points(points, routeStartPoint=None, routeEndPoint=None, attractor=None):

    # STEP 1: Adds the routeStartPoint (it will be deleted at the end) and define the attractor
    if routeStartPoint is None:
        route = [{'x': 0, 'y': 0}]
    else:
        route = [{'x': routeStartPoint[0], 'y': routeStartPoint[1]}]

    if attractor is None:
        attractor = [route[0]['x'], route[0]['y']]

    # STEP 2: Applies nearest neighbour algorithm
    while points:
        distBest = float('inf')
        for neighbour in points:
            distLP2N = math.dist([route[-1]['x'],route[-1]['y']], [neighbour['x'],neighbour['y']])
            if distLP2N < distBest - 1e-6:
                # "neighbour" is closer to last point than the current "nearestNeighbour"
                distBest = distLP2N
                nearestNeighbour = neighbour
            elif distLP2N < distBest + 1e-6:
                # both points are the same distance away from last point -> choose the point which is closer to the attractor
                distA2N = math.dist([attractor[0],attractor[1]], [neighbour['x'],neighbour['y']])
                distA2NN = math.dist([attractor[0],attractor[1]], [nearestNeighbour['x'],nearestNeighbour['y']])
                if distA2N < distA2NN - 1e-6:
                    nearestNeighbour = neighbour
                elif distA2N < distA2NN + 1e-6:
                    # both points are the same distance away from the attractor -> choose the point with bigger x coordinate
                    if neighbour['x'] > nearestNeighbour['x'] + 1e-6:
                        nearestNeighbour = neighbour
                    elif neighbour['x'] > nearestNeighbour['x'] - 1e-6:
                        # both points have the same x coordinate -> choose the point with bigger y coordinate
                        if neighbour['y'] > nearestNeighbour['y']:
                            nearestNeighbour = neighbour
        route.append(nearestNeighbour)
        points.remove(nearestNeighbour)

    # STEP 3: Adds the routeEndPoint (it will be deleted at the end)
    if routeEndPoint is not None:
        route.append({'x': routeEndPoint[0], 'y': routeEndPoint[1]})

    # STEP 4: Additional improvement of the route
    lengthRoute = len(route)
    
    limitRelocationI = lengthRoute - 1
    limitReorderI = lengthRoute - 2
    limitReorderJ = lengthRoute + 1
    if routeEndPoint is not None:
        limitRelocationI -= 1
        limitReorderI -= 1
        limitReorderJ -= 1

    lastImprovementAtStep = 0 

    while True:

        # STEP 4.1: Applies relocation
        if lastImprovementAtStep == 1: break
        improvementFound = True
        while improvementFound:
            improvementFound = False
            # let's try to relocate the i-th point...
            for i in range(limitRelocationI,0,-1):
                deltaBest = 0
                subRouteLengthCurrent = math.dist([route[i-1]['x'],route[i-1]['y']], [route[i]['x'],route[i]['y']])
                if i + 1 < lengthRoute:
                    subRouteLengthCurrent += math.dist([route[i]['x'],route[i]['y']], [route[i+1]['x'],route[i+1]['y']])
                    subRouteLengthCurrent -= math.dist([route[i-1]['x'],route[i-1]['y']], [route[i+1]['x'],route[i+1]['y']])
                # ...after the j-th point...
                for j in range(i-1):
                    subRouteLengthNew = math.dist([route[j]['x'],route[j]['y']], [route[i]['x'],route[i]['y']])
                    subRouteLengthNew += math.dist([route[i]['x'],route[i]['y']], [route[j+1]['x'],route[j+1]['y']])
                    subRouteLengthNew -= math.dist([route[j]['x'],route[j]['y']], [route[j+1]['x'],route[j+1]['y']])
                    delta = subRouteLengthNew - subRouteLengthCurrent
                    # ...and see if there is an improvement
                    if delta < deltaBest - 1e-6:
                        # improvement found!
                        deltaBest = delta
                        jBest = j
                if deltaBest < 0:
                    route.insert(jBest+1, route.pop(i)) # relocate the i-th point backward (after jBest-th point)
                    improvementFound = True
                    lastImprovementAtStep = 1

        # STEP 4.2: Applies reorder (2-opt)
        if lastImprovementAtStep == 2: break
        improvementFound = True
        while improvementFound:
            improvementFound = False
            # let's try to reverse the order of points between the i-th point...
            for i in range(limitReorderI):
                deltaBest = 0
                subRouteLengthCurrentPart = math.dist([route[i]['x'],route[i]['y']], [route[i+1]['x'],route[i+1]['y']])
                # ...and the j-th point...
                for j in range(i+3,limitReorderJ):
                    subRouteLengthCurrent = subRouteLengthCurrentPart
                    subRouteLengthNew = math.dist([route[i]['x'],route[i]['y']], [route[j-1]['x'],route[j-1]['y']])
                    if j < lengthRoute:
                        subRouteLengthCurrent += math.dist([route[j-1]['x'],route[j-1]['y']], [route[j]['x'],route[j]['y']])
                        subRouteLengthNew += math.dist([route[i+1]['x'],route[i+1]['y']], [route[j]['x'],route[j]['y']])
                    delta = subRouteLengthNew - subRouteLengthCurrent
                    # ...and see if there is an improvement
                    if delta < deltaBest - 1e-6:
                        # improvement found!
                        deltaBest = delta
                        jBest = j
                if deltaBest < 0:
                    route[i+1:jBest] = route[i+1:jBest][::-1] # reverse the order of points between i-th and jBest-th point
                    improvementFound = True
                    lastImprovementAtStep = 2

        if lastImprovementAtStep == 0: break # no additional improvementes could be made

    # STEP 5: Deletes temporary start and end point
    del route[0]
    if routeEndPoint is not None:
        del route[-1]

    return route


# --- code below this line is just to demonstrate the capability of tsp_solver_points() function ---

import random
import matplotlib.pyplot as plt
import time

# STEP 1: Generate list of points (1=generate ; 0=don't generate)
genPointsOne1 = 0 # 1 point, random position
genPointsEq10 = 1 # 100 points, equidistant 10x10 grid
genPointsEq45 = 0 # 98 points, equidistant grid 45 degree
genPointsEqHx = 0 # 90 points, equidistant hexagonal grid
genPointsPola = 0 # 100 points, polar pattern
genPointsRand = 0 # 100 points, random position

points = []

if genPointsOne1:
    points.append({'x': 100*random.random(), 'y': 100*random.random()})

if genPointsEq10:
    for i in range(10):
        for j in range(10):
            points.append({'x': 10*i + 5, 'y': 10*j + 5})

if genPointsEq45:
    for i in range(7):
        for j in range(7):
            points.append({'x': 14*i + 5, 'y': 14*j + 5})
            points.append({'x': 14*i + 12, 'y': 14*j + 12})

if genPointsEqHx:
    for i in range(5):
        for j in range(9):
            points.append({'x': 18*i + 10, 'y': 18*j/math.sqrt(3) + 5})
            points.append({'x': 18*i + 19, 'y': 18*j/math.sqrt(3) + 9/math.sqrt(3) + 5})

if genPointsPola:
    for i in range(9):
        points.append({'x': 50 + 5*math.cos(i*2*math.pi/9), 'y': 50 + 5*math.sin(i*2*math.pi/9)})
    for i in range(14):
        points.append({'x': 50 + 15*math.cos(i*2*math.pi/14), 'y': 50 + 15*math.sin(i*2*math.pi/14)})
    for i in range(19):
        points.append({'x': 50 + 25*math.cos(i*2*math.pi/19), 'y': 50 + 25*math.sin(i*2*math.pi/19)})
    for i in range(25):
        points.append({'x': 50 + 35*math.cos(i*2*math.pi/25), 'y': 50 + 35*math.sin(i*2*math.pi/25)})
    for i in range(33):
        points.append({'x': 50 + 45*math.cos(i*2*math.pi/33), 'y': 50 + 45*math.sin(i*2*math.pi/33)})

if genPointsRand:
    for i in range(100):
        points.append({'x': 100*random.random(), 'y': 100*random.random()})

# STEP 2: Define start point, end point and attractor for each example
routeStartPoint = [[None for j in range(4)] for i in range(2)]
routeEndPoint = [[None for j in range(4)] for i in range(2)]
attractor = [[None for j in range(4)] for i in range(2)]

routeStartPoint[0][0] = [0,0]
routeEndPoint[0][0] = None
attractor[0][0] = None

routeStartPoint[0][1] = [0,50]
routeEndPoint[0][1] = None
attractor[0][1] = None

routeStartPoint[0][2] = [50,50]
routeEndPoint[0][2] = None
attractor[0][2] = None

routeStartPoint[0][3] = [100,50]
routeEndPoint[0][3] = None
attractor[0][3] = None

routeStartPoint[1][0] = [0,0]
routeEndPoint[1][0] = [100,100]
attractor[1][0] = None

routeStartPoint[1][1] = [0,50]
routeEndPoint[1][1] = [0,50]
attractor[1][1] = None

routeStartPoint[1][2] = [50,50]
routeEndPoint[1][2] = [0,100]
attractor[1][2] = None

routeStartPoint[1][3] = [100,50]
routeEndPoint[1][3] = [50,50]
attractor[1][3] = None

# STEP 3: Find efficient route and plot it
fig, axs = plt.subplots(2,4)
sumTime = 0
sumLength = 0

for r in range(2):
    for c in range(4):

        # shuffle the list of points before sorting
        points = random.sample(points, len(points))

        # run the solver and measure the time needed
        print('Solving row ' + str(r+1) + ', column ' + str(c+1) + '...')
        timeStart = time.time()
        points = tsp_solver_points(points, routeStartPoint[r][c], routeEndPoint[r][c], attractor[r][c])
        timeEnd = time.time()
        timeDelta = timeEnd - timeStart
        timeDelta = round(1000 * timeDelta) # convert to miliseconds
        sumTime += timeDelta

        # calculate total length
        totalLength = 0
        for i in range(len(points)-1):
            totalLength += math.dist([points[i]['x'],points[i]['y']], [points[i+1]['x'],points[i+1]['y']])
        totalLength = round(totalLength)
        sumLength += totalLength

        # draw points
        for i in range(len(points)):
            axs[r,c].scatter(points[i]['x'], points[i]['y'], color='b', marker='.')

        # draw route
        for i in range(len(points)-1):
            axs[r,c].plot([points[i]['x'], points[i+1]['x']], [points[i]['y'], points[i+1]['y']], color='r')

        # draw path from routeStartPoint
        axs[r,c].scatter(routeStartPoint[r][c][0], routeStartPoint[r][c][1], color='b', marker='>')
        axs[r,c].plot([routeStartPoint[r][c][0], points[0]['x']], [routeStartPoint[r][c][1], points[0]['y']], color='r', linestyle='dashed')

        # draw path to routeEndPoint
        if routeEndPoint[r][c] is not None:
            axs[r,c].scatter(routeEndPoint[r][c][0], routeEndPoint[r][c][1], color='b', marker='s')
            axs[r,c].plot([points[-1]['x'], routeEndPoint[r][c][0]], [points[-1]['y'], routeEndPoint[r][c][1]], color='r', linestyle='dashed')

        fig.suptitle('Number of points: ' + str(len(points)))
        axs[r,c].set_title('SP=' + str(routeStartPoint[r][c]) + ' | EP=' + str(routeEndPoint[r][c]) + ' | t=' + str(timeDelta) + 'ms' + ' | l=' + str(totalLength) + ' | A=' + str(attractor[r][c]), fontsize=10)
        axs[r,c].set_xlim([-5, 105])
        axs[r,c].set_ylim([-5, 105])
        axs[r,c].set_aspect('equal')

print(sumTime, 'ms')
print(sumLength)
plt.show()
