import math
import random
import matplotlib.pyplot as plt
import time

def tsp_solver_points(points, routeStartPoint=None, routeEndPoint=None):

    # STEP 1: Adds the routeStartPoint (it will be deleted at the end)
    if routeStartPoint is None:
        points.insert(0,{'x': 0, 'y': 0})
    else:
        points.insert(0,{'x': routeStartPoint[0], 'y': routeStartPoint[1]})

    # STEP 2: Applies nearest neighbour algorithm
    potentialNeighbours = points[1:]
    route = [points[0]]
    while potentialNeighbours:
        costCurrent = float('inf')
        for neighbour in potentialNeighbours:
            costNew = (route[-1]['x'] - neighbour['x'])**2 + (route[-1]['y'] - neighbour['y'])**2
            if costNew > costCurrent + 1e-3:
                # point is further than the current best point
                continue
            elif costNew < costCurrent - 1e-3:
                # point is closer than the current best point
                costCurrent = costNew
                nearestNeighbour = neighbour
            else:
                # there is a tie, choose the point which is closest to the routeStartPoint
                if (route[0]['x'] - neighbour['x'])**2 + (route[0]['y'] - neighbour['y'])**2 < (route[0]['x'] - nearestNeighbour['x'])**2 + (route[0]['y'] - nearestNeighbour['y'])**2:
                    costCurrent = costNew
                    nearestNeighbour = neighbour
        route.append(nearestNeighbour)
        potentialNeighbours.remove(nearestNeighbour)

    # STEP 3: Adds the routeEndPoint (it will be deleted at the end)
    if routeEndPoint is not None:
        route.append({'x': routeEndPoint[0], 'y': routeEndPoint[1]})

    # STEP 4: Additional improvement of the route
    limitReorderI = len(route) - 2
    if routeEndPoint is not None:
        limitReorderI -= 1
    limitReorderJ = len(route)
    limitRelocationI = len(route) - 1
    limitRelocationJ = len(route) - 1
    lastImprovementAtStep = 0 

    while True:

        # STEP 4.1: Applies 2-opt
        if lastImprovementAtStep == 1: break
        improvementFound = True
        while improvementFound:
            improvementFound = False
            for i in range(0,limitReorderI):
                subRouteLengthCurrentPart = math.dist([route[i]['x'],route[i]['y']], [route[i+1]['x'],route[i+1]['y']])
                for j in range(i+3,limitReorderJ):
                    subRouteLengthCurrent = subRouteLengthCurrentPart
                    subRouteLengthCurrent += math.dist([route[j-1]['x'],route[j-1]['y']], [route[j]['x'],route[j]['y']])
                    subRouteLengthNew = math.dist([route[i+1]['x'],route[i+1]['y']], [route[j]['x'],route[j]['y']])
                    subRouteLengthNew += math.dist([route[i]['x'],route[i]['y']], [route[j-1]['x'],route[j-1]['y']])
                    subRouteLengthNew += 1e-6
                    if subRouteLengthNew < subRouteLengthCurrent:
                        route[i+1:j] = route[i+1:j][::-1] # reverse the order of points between i-th and j-th point
                        subRouteLengthCurrentPart = math.dist([route[i]['x'],route[i]['y']], [route[i+1]['x'],route[i+1]['y']])
                        improvementFound = True
                        lastImprovementAtStep = 1
                if routeEndPoint is None:
                    subRouteLengthCurrent = math.dist([route[i]['x'],route[i]['y']], [route[i+1]['x'],route[i+1]['y']])
                    subRouteLengthNew = math.dist([route[i]['x'],route[i]['y']], [route[-1]['x'],route[-1]['y']])
                    subRouteLengthNew += 1e-6
                    if subRouteLengthNew < subRouteLengthCurrent:
                        route[i+1:limitReorderJ] = route[i+1:limitReorderJ][::-1] # reverse the order of points after i-th to the last point
                        improvementFound = True
                        lastImprovementAtStep = 1

        # STEP 4.2: Applies relocation
        if lastImprovementAtStep == 2: break
        improvementFound = True
        while improvementFound:
            improvementFound = False
            for i in range(1,limitRelocationI):
                subRouteLengthCurrentPart = math.dist([route[i-1]['x'],route[i-1]['y']], [route[i]['x'],route[i]['y']])
                subRouteLengthCurrentPart += math.dist([route[i]['x'],route[i]['y']], [route[i+1]['x'],route[i+1]['y']])
                subRouteLengthNewPart = math.dist([route[i-1]['x'],route[i-1]['y']], [route[i+1]['x'],route[i+1]['y']])
                subRouteLengthNewPart += 1e-6
                for j in range(0,i-2):
                    subRouteLengthCurrent = subRouteLengthCurrentPart
                    subRouteLengthCurrent += math.dist([route[j]['x'],route[j]['y']], [route[j+1]['x'],route[j+1]['y']])
                    subRouteLengthNew = subRouteLengthNewPart
                    subRouteLengthNew += math.dist([route[j]['x'],route[j]['y']], [route[i]['x'],route[i]['y']])
                    subRouteLengthNew += math.dist([route[i]['x'],route[i]['y']], [route[j+1]['x'],route[j+1]['y']])
                    if subRouteLengthNew < subRouteLengthCurrent:
                        route.insert(j+1, route.pop(i)) # relocate the i-th point backward (after j-th point)
                        subRouteLengthCurrentPart = math.dist([route[i-1]['x'],route[i-1]['y']], [route[i]['x'],route[i]['y']])
                        subRouteLengthCurrentPart += math.dist([route[i]['x'],route[i]['y']], [route[i+1]['x'],route[i+1]['y']])
                        subRouteLengthNewPart = math.dist([route[i-1]['x'],route[i-1]['y']], [route[i+1]['x'],route[i+1]['y']])
                        subRouteLengthNewPart += 1e-6
                        improvementFound = True
                        lastImprovementAtStep = 2
                for j in range(i+1,limitRelocationJ):
                    subRouteLengthCurrent = subRouteLengthCurrentPart
                    subRouteLengthCurrent += math.dist([route[j]['x'],route[j]['y']], [route[j+1]['x'],route[j+1]['y']])
                    subRouteLengthNew = subRouteLengthNewPart
                    subRouteLengthNew += math.dist([route[j]['x'],route[j]['y']], [route[i]['x'],route[i]['y']])
                    subRouteLengthNew += math.dist([route[i]['x'],route[i]['y']], [route[j+1]['x'],route[j+1]['y']])
                    if subRouteLengthNew < subRouteLengthCurrent:
                        route.insert(j, route.pop(i)) # relocate the i-th point forward (after j-th point)
                        subRouteLengthCurrentPart = math.dist([route[i-1]['x'],route[i-1]['y']], [route[i]['x'],route[i]['y']])
                        subRouteLengthCurrentPart += math.dist([route[i]['x'],route[i]['y']], [route[i+1]['x'],route[i+1]['y']])
                        subRouteLengthNewPart = math.dist([route[i-1]['x'],route[i-1]['y']], [route[i+1]['x'],route[i+1]['y']])
                        subRouteLengthNewPart += 1e-6
                        improvementFound = True
                        lastImprovementAtStep = 2
            if routeEndPoint is None:
                subRouteLengthCurrentPart = math.dist([route[-2]['x'],route[-2]['y']], [route[-1]['x'],route[-1]['y']])
                for j in range(0,len(route)-2):
                    subRouteLengthCurrent = subRouteLengthCurrentPart
                    subRouteLengthCurrent += math.dist([route[j]['x'],route[j]['y']], [route[j+1]['x'],route[j+1]['y']])
                    subRouteLengthNew = math.dist([route[j]['x'],route[j]['y']], [route[-1]['x'],route[-1]['y']])
                    subRouteLengthNew += math.dist([route[-1]['x'],route[-1]['y']], [route[j+1]['x'],route[j+1]['y']])
                    subRouteLengthNew += 1e-6
                    if subRouteLengthNew < subRouteLengthCurrent:
                        route.insert(j+1, route.pop(-1)) # relocate the last point after j-th point
                        subRouteLengthCurrentPart = math.dist([route[-2]['x'],route[-2]['y']], [route[-1]['x'],route[-1]['y']])
                        improvementFound = True
                        lastImprovementAtStep = 2

        if lastImprovementAtStep == 0: break # no additional improvementes could be made

    # STEP 5: Deletes temporary start and end point
    del route[0]
    if routeEndPoint is not None:
        del route[-1]

    return route


# --- code below this line is just to demonstrate the capability of tsp_solver_points() function ---


# STEP 1: Generate list of points (1=generate ; 0=don't generate)
genPointsEq10 = 1 # 100 points, equidistant 10x10 grid
genPointsEq45 = 0 # 98 points, equidistant grid 45 degree
genPointsEqHx = 0 # 90 points, equidistant hexagonal grid
genPointsPola = 0 # 100 points, polar pattern
genPointsRand = 0 # 100 points, random position

points = []

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

# STEP 2: Define start and end point for each example
routeStartPoint = [[None for j in range(4)] for i in range(2)]
routeEndPoint = [[None for j in range(4)] for i in range(2)]

routeStartPoint[0][0] = [0,0]
routeEndPoint[0][0] = None

routeStartPoint[0][1] = [0,50]
routeEndPoint[0][1] = None

routeStartPoint[0][2] = [50,50]
routeEndPoint[0][2] = None

routeStartPoint[0][3] = [100,50]
routeEndPoint[0][3] = None

routeStartPoint[1][0] = [0,0]
routeEndPoint[1][0] = [100,100]

routeStartPoint[1][1] = [0,50]
routeEndPoint[1][1] = [0,50]

routeStartPoint[1][2] = [50,50]
routeEndPoint[1][2] = [0,100]

routeStartPoint[1][3] = [100,50]
routeEndPoint[1][3] = [50,50]

# STEP 3: Find efficient route and plot it
fig, axs = plt.subplots(2,4)

for r in range(2):
    for c in range(4):

        # shuffle the list of points before sorting
        points = random.sample(points, len(points))

        # run the solver and measure the time needed
        print('Solving row ' + str(r+1) + ', column ' + str(c+1) + '...')
        timeStart = time.time()
        points = tsp_solver_points(points, routeStartPoint[r][c], routeEndPoint[r][c])
        timeEnd = time.time()
        timeDelta = timeEnd - timeStart
        timeDelta = round(1000 * timeDelta) # convert to miliseconds

        # calculate total length
        totalLength = 0
        for i in range(len(points)-1):
            totalLength += math.dist([points[i]['x'],points[i]['y']], [points[i+1]['x'],points[i+1]['y']])
        totalLength = round(totalLength)

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
        axs[r,c].set_title('SP=' + str(routeStartPoint[r][c]) + ' | EP=' +str(routeEndPoint[r][c]) + ' | t=' + str(timeDelta) + 'ms' + ' | l=' + str(totalLength), fontsize=10)
        axs[r,c].set_xlim([-5, 105])
        axs[r,c].set_ylim([-5, 105])
        axs[r,c].set_aspect('equal')

plt.show()
