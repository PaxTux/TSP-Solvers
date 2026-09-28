import math

def tsp_solver_tunnels(tunnels, routeStartPoint=None, routeEndPoint=None, allowFlipping=False):

    # STEP 1: Adds the routeStartPoint (it will be deleted at the end)
    if routeStartPoint is None:
        route = [{'startX': 0, 'startY': 0, 'endX': 0, 'endY': 0}]
    else:
        route = [{'startX': routeStartPoint[0], 'startY': routeStartPoint[1], 'endX': routeStartPoint[0], 'endY': routeStartPoint[1]}]

    # STEP 2: Nearest neighbour algorithm
    while tunnels:
        distBest = float('inf')
        for neighbour in tunnels:
            distLP2N = math.dist([route[-1]['endX'],route[-1]['endY']], [neighbour['startX'],neighbour['startY']])
            if distLP2N < distBest - 1e-6:
                # "neighbour" is closer to last point than the current "nearestNeighbour"
                distBest = distLP2N
                nearestNeighbour = neighbour
                toBeFlipped = False
            elif distLP2N < distBest + 1e-6:
                # both points are the same distance away from last point -> choose the point which is closer to the target
                target = [2*route[-1]['endX'] - route[-1]['startX'] , 2*route[-1]['endY'] - route[-1]['startY']]
                distT2N = math.dist([target[0],target[1]], [neighbour['endX'],neighbour['endY']])
                distT2NN = math.dist([target[0],target[1]], [nearestNeighbour['endX'],nearestNeighbour['endY']])
                if distT2N < distT2NN - 1e-6:
                    nearestNeighbour = neighbour
                    toBeFlipped = False
                elif distT2N < distT2NN + 1e-6:
                    # both points are the same distance away from the target -> choose the point with bigger x coordinate
                    if neighbour['startX'] > nearestNeighbour['startX'] + 1e-6:
                        nearestNeighbour = neighbour
                        toBeFlipped = False
                    elif neighbour['startX'] > nearestNeighbour['startX'] - 1e-6:
                        # both points have the same x coordinate -> choose the point with bigger y coordinate
                        if neighbour['startY'] > nearestNeighbour['startY']:
                            nearestNeighbour = neighbour
                            toBeFlipped = False
        if allowFlipping:
            for neighbour in tunnels:
                distLP2N = math.dist([route[-1]['endX'],route[-1]['endY']], [neighbour['endX'],neighbour['endY']])
                if distLP2N < distBest - 1e-6:
                    # "neighbour" is closer to last point than the current "nearestNeighbour"
                    distBest = distLP2N
                    nearestNeighbour = neighbour
                    toBeFlipped = True
                elif distLP2N < distBest + 1e-6:
                    # both points are the same distance away from last point -> choose the point which is closer to the target
                    target = [2*route[-1]['endX'] - route[-1]['startX'] , 2*route[-1]['endY'] - route[-1]['startY']]
                    distT2N = math.dist([target[0],target[1]], [neighbour['startX'],neighbour['startY']])
                    distT2NN = math.dist([target[0],target[1]], [nearestNeighbour['startX'],nearestNeighbour['startY']])
                    if distT2N < distT2NN - 1e-6:
                        nearestNeighbour = neighbour
                        toBeFlipped = True
                    elif distT2N < distT2NN + 1e-6:
                        # both points are the same distance away from the target -> choose the point with bigger x coordinate
                        if neighbour['endX'] > nearestNeighbour['endX'] + 1e-6:
                            nearestNeighbour = neighbour
                            toBeFlipped = True
                        elif neighbour['endX'] > nearestNeighbour['endX'] - 1e-6:
                            # both points have the same x coordinate -> choose the point with bigger y coordinate
                            if neighbour['endY'] > nearestNeighbour['endY']:
                                nearestNeighbour = neighbour
                                toBeFlipped = True
        tunnels.remove(nearestNeighbour)
        if toBeFlipped:
            nearestNeighbour['startX'], nearestNeighbour['endX'] = nearestNeighbour['endX'], nearestNeighbour['startX']
            nearestNeighbour['startY'], nearestNeighbour['endY'] = nearestNeighbour['endY'], nearestNeighbour['startY']
        route.append(nearestNeighbour)

    # STEP 3: Adds the routeEndPoint (it will be deleted at the end)
    if routeEndPoint is not None:
        route.append({'startX': routeEndPoint[0], 'startY': routeEndPoint[1]})

    # STEP 4: Additional improvement of the route
    lengthRoute = len(route)

    limitRelocationI = lengthRoute - 1
    limitRelocationJ = lengthRoute
    limitReorderI = lengthRoute - 2
    limitReorderJ = lengthRoute + 1
    limitFlipI = lengthRoute
    if routeEndPoint is not None:
        limitRelocationI -= 1
        limitRelocationJ -= 1
        limitReorderI -= 1
        limitReorderJ -= 1
        limitFlipI -= 1

    lastImprovementAtStep = 0

    while True:

        # STEP 4.1: Flipping
        if allowFlipping:
            if lastImprovementAtStep == 1: break
            improvementFound = True
            while improvementFound:
                improvementFound = False
                # let's try to flip the i-th tunnel...
                for i in range(1,limitFlipI):
                    subRouteLengthCurrent = math.dist([route[i-1]['endX'],route[i-1]['endY']], [route[i]['startX'],route[i]['startY']])
                    subRouteLengthNew = math.dist([route[i-1]['endX'],route[i-1]['endY']], [route[i]['endX'],route[i]['endY']])
                    if i + 1 < lengthRoute:
                        subRouteLengthCurrent += math.dist([route[i]['endX'],route[i]['endY']], [route[i+1]['startX'],route[i+1]['startY']])
                        subRouteLengthNew += math.dist([route[i]['startX'],route[i]['startY']], [route[i+1]['startX'],route[i+1]['startY']])
                    delta = subRouteLengthNew - subRouteLengthCurrent
                    # ...and see if there is an improvement
                    if delta < -1e-6:
                        # improvement found!
                        # flips direction of i-th tunnel
                        route[i]['startX'], route[i]['endX'] = route[i]['endX'], route[i]['startX']
                        route[i]['startY'], route[i]['endY'] = route[i]['endY'], route[i]['startY']
                        improvementFound = True
                        lastImprovementAtStep = 1
                    elif delta < 1e-6:
                        # no change, but still flips direction of i-th tunnel
                        route[i]['startX'], route[i]['endX'] = route[i]['endX'], route[i]['startX']
                        route[i]['startY'], route[i]['endY'] = route[i]['endY'], route[i]['startY']

        # STEP 4.2: Relocation backward
        if lastImprovementAtStep == 2: break
        improvementFound = True
        while improvementFound:
            improvementFound = False
            # let's try to relocate the i-th tunnel backward...
            for i in range(limitRelocationI,0,-1):
                deltaBest = 0
                subRouteLengthCurrent = math.dist([route[i-1]['endX'],route[i-1]['endY']], [route[i]['startX'],route[i]['startY']])
                if i + 1 < lengthRoute:
                    subRouteLengthCurrent += math.dist([route[i]['endX'],route[i]['endY']], [route[i+1]['startX'],route[i+1]['startY']])
                    subRouteLengthCurrent -= math.dist([route[i-1]['endX'],route[i-1]['endY']], [route[i+1]['startX'],route[i+1]['startY']])
                # ...after the j-th tunnel...
                for j in range(i-1):
                    subRouteLengthNew = math.dist([route[j]['endX'],route[j]['endY']], [route[i]['startX'],route[i]['startY']])
                    subRouteLengthNew += math.dist([route[i]['endX'],route[i]['endY']], [route[j+1]['startX'],route[j+1]['startY']])
                    subRouteLengthNew -= math.dist([route[j]['endX'],route[j]['endY']], [route[j+1]['startX'],route[j+1]['startY']])
                    delta = subRouteLengthNew - subRouteLengthCurrent
                    # ...and see if there is an improvement
                    if delta < deltaBest - 1e-6:
                        # improvement found!
                        deltaBest = delta
                        jBest = j
                if deltaBest < 0:
                    route.insert(jBest+1, route.pop(i)) # relocate the i-th tunnel backward (after jBest-th tunnel)
                    improvementFound = True
                    lastImprovementAtStep = 2

        # STEP 4.3: Reorder (2-opt)
        if allowFlipping:
            if lastImprovementAtStep == 3: break
            improvementFound = True
            while improvementFound:
                improvementFound = False
                # let's try to reverse the order of tunnels between the i-th tunnel...
                for i in range(limitReorderI):
                    deltaBest = 0
                    subRouteLengthCurrentPart = math.dist([route[i]['endX'],route[i]['endY']], [route[i+1]['startX'],route[i+1]['startY']])
                    # ...and the j-th tunnel...
                    for j in range(i+3,limitReorderJ):
                        subRouteLengthCurrent = subRouteLengthCurrentPart
                        subRouteLengthNew = math.dist([route[i]['endX'],route[i]['endY']], [route[j-1]['endX'],route[j-1]['endY']])
                        if j < lengthRoute:
                            subRouteLengthCurrent += math.dist([route[j-1]['endX'],route[j-1]['endY']], [route[j]['startX'],route[j]['startY']])
                            subRouteLengthNew += math.dist([route[i+1]['startX'],route[i+1]['startY']], [route[j]['startX'],route[j]['startY']])
                        delta = subRouteLengthNew - subRouteLengthCurrent
                        # ...and see if there is an improvement
                        if delta < deltaBest - 1e-6:
                            # improvement found!
                            deltaBest = delta
                            jBest = j
                    if deltaBest < 0:
                        for k in range(i+1,jBest): # flips direction of each tunnel between i-th and jBest-th tunnel
                            route[k]['startX'], route[k]['endX'] = route[k]['endX'], route[k]['startX']
                            route[k]['startY'], route[k]['endY'] = route[k]['endY'], route[k]['startY']
                        route[i+1:jBest] = route[i+1:jBest][::-1] # reverse the order of tunnels between i-th and jBest-th tunnel
                        improvementFound = True
                        lastImprovementAtStep = 3

        # STEP 4.4: Relocation forward
        if lastImprovementAtStep == 4: break
        improvementFound = True
        while improvementFound:
            improvementFound = False
            # let's try to relocate the i-th tunnel forward...
            for i in range(limitRelocationI,0,-1):
                deltaBest = 0
                subRouteLengthCurrent = math.dist([route[i-1]['endX'],route[i-1]['endY']], [route[i]['startX'],route[i]['startY']])
                if i + 1 < lengthRoute:
                    subRouteLengthCurrent += math.dist([route[i]['endX'],route[i]['endY']], [route[i+1]['startX'],route[i+1]['startY']])
                    subRouteLengthCurrent -= math.dist([route[i-1]['endX'],route[i-1]['endY']], [route[i+1]['startX'],route[i+1]['startY']])
                # ...after the j-th tunnel...
                for j in range(i+1,limitRelocationJ):
                    subRouteLengthNew = math.dist([route[j]['endX'],route[j]['endY']], [route[i]['startX'],route[i]['startY']])
                    if j + 1 < lengthRoute:
                        subRouteLengthNew += math.dist([route[i]['endX'],route[i]['endY']], [route[j+1]['startX'],route[j+1]['startY']])
                        subRouteLengthNew -= math.dist([route[j]['endX'],route[j]['endY']], [route[j+1]['startX'],route[j+1]['startY']])
                    delta = subRouteLengthNew - subRouteLengthCurrent
                    # ...and see if there is an improvement
                    if delta < deltaBest - 1e-6:
                        # improvement found!
                        deltaBest = delta
                        jBest = j
                if deltaBest < 0:
                    route.insert(jBest, route.pop(i)) # relocate the i-th tunnel forward (after jBest-th tunnel)
                    improvementFound = True
                    lastImprovementAtStep = 4

        if lastImprovementAtStep == 0: break # no additional improvementes could be made

    # STEP 5: Deletes temporary start and end point
    del route[0]
    if routeEndPoint is not None:
        del route[-1]

    return route


# --- code below this line is just to demonstrate the capability of tsp_solver_tunnels() function ---

import random
import copy
import time
import matplotlib.pyplot as plt

# STEP 1: Generate the list of tunnels (1=generate ; 0=don't generate)
genTunnelsOne1 = 0 # 1 tunnel, random position
genTunnelsHori = 1 # 50 tunnels, horizontal pattern
genTunnelsVert = 0 # 50 tunnels, vertical pattern
genTunnelsLong = 0 # 20 tunnels, 10 long horizontal and 10 long vertical
genTunnelsRand = 0 # 20 tunnels, random position

generatedTunnels = []

if genTunnelsOne1:
    generatedTunnels.append({'startX': 100*random.random(), 'startY': 100*random.random(), 'endX': 100*random.random(), 'endY': 100*random.random()})

if genTunnelsHori:
    for i in range(5):
        for j in range(10):
            generatedTunnels.append({'startX': 20*i+5, 'startY': 10*j+5, 'endX': 20*i+15, 'endY': 10*j+5})

if genTunnelsVert:
    for i in range(5):
        for j in range(10):
            generatedTunnels.append({'startX': 10*j+5, 'startY': 20*i+5, 'endX': 10*j+5, 'endY': 20*i+15})

if genTunnelsLong:
    for i in range(10):
        generatedTunnels.append({'startX': 0, 'startY': 10*i+5, 'endX': 100, 'endY': 10*i+5})
    for i in range(10):
        generatedTunnels.append({'startX': 10*i+5, 'startY': 0, 'endX': 10*i+5, 'endY': 100})

if genTunnelsRand:
    for i in range(20):
        generatedTunnels.append({'startX': 100*random.random(), 'startY': 100*random.random(), 'endX': 100*random.random(), 'endY': 100*random.random()})

generatedTunnels = random.sample(generatedTunnels, len(generatedTunnels)) # shuffle the list of tunnels

# STEP 2: Define start point, end point and allowFlipping for each example
routeStartPoint = [[None for j in range(4)] for i in range(2)]
routeEndPoint = [[None for j in range(4)] for i in range(2)]
allowFlipping = [[None for j in range(4)] for i in range(2)]

routeStartPoint[0][0] = None
routeEndPoint[0][0] = None
allowFlipping[0][0] = True

routeStartPoint[0][1] = [0,50]
routeEndPoint[0][1] = None
allowFlipping[0][1] = True

routeStartPoint[0][2] = [50,50]
routeEndPoint[0][2] = None
allowFlipping[0][2] = True

routeStartPoint[0][3] = [100,50]
routeEndPoint[0][3] = None
allowFlipping[0][3] = True

routeStartPoint[1][0] = None
routeEndPoint[1][0] = [0,100]
allowFlipping[1][0] = True

routeStartPoint[1][1] = None
routeEndPoint[1][1] = [50,50]
allowFlipping[1][1] = True

routeStartPoint[1][2] = None
routeEndPoint[1][2] = [0,0]
allowFlipping[1][2] = True

routeStartPoint[1][3] = None
routeEndPoint[1][3] = [100,100]
allowFlipping[1][3] = True

# STEP 3: Find efficient route and plot it
fig, axs = plt.subplots(2,4)

for r in range(2):
    for c in range(4):

        tunnels = copy.deepcopy(generatedTunnels)

        # run the solver and measure the time needed
        print('Solving row ' + str(r+1) + ', column ' + str(c+1) + '...')
        timeStart = time.time()
        tunnels = tsp_solver_tunnels(tunnels, routeStartPoint[r][c], routeEndPoint[r][c], allowFlipping[r][c])
        timeEnd = time.time()
        timeDelta = timeEnd - timeStart
        timeDelta = round(1000 * timeDelta) # convert to miliseconds

        # calculate total length
        totalLength = 0
        for i in range(len(tunnels)-1):
            totalLength += math.dist([tunnels[i]['endX'],tunnels[i]['endY']], [tunnels[i+1]['startX'],tunnels[i+1]['startY']])
        totalLength = round(totalLength)

        # draw tunnels
        for i in range(len(tunnels)):
            axs[r,c].scatter(tunnels[i]['startX'], tunnels[i]['startY'], color='b', marker='.')
            axs[r,c].scatter(tunnels[i]['endX'], tunnels[i]['endY'], color='b', marker='.')
            axs[r,c].plot([tunnels[i]['startX'], tunnels[i]['endX']], [tunnels[i]['startY'], tunnels[i]['endY']], linewidth=1, color='b', linestyle='dashed')

        # draw route
        for i in range(len(tunnels)-1):
            axs[r,c].plot([tunnels[i]['endX'], tunnels[i+1]['startX']], [tunnels[i]['endY'], tunnels[i+1]['startY']], color='r')

        # draw path from routeStartPoint
        if routeStartPoint[r][c] is None:
            axs[r,c].scatter(0, 0, color='b', marker='>')
            axs[r,c].plot([0, tunnels[0]['startX']], [0, tunnels[0]['startY']], color='r', linestyle='dashed')
        else:
            axs[r,c].scatter(routeStartPoint[r][c][0], routeStartPoint[r][c][1], color='b', marker='>')
            axs[r,c].plot([routeStartPoint[r][c][0], tunnels[0]['startX']], [routeStartPoint[r][c][1], tunnels[0]['startY']], color='r', linestyle='dashed')

        # draw path to routeEndPoint
        if routeEndPoint[r][c] is not None:
            axs[r,c].scatter(routeEndPoint[r][c][0], routeEndPoint[r][c][1], color='b', marker='s')
            axs[r,c].plot([tunnels[-1]['endX'], routeEndPoint[r][c][0]], [tunnels[-1]['endY'], routeEndPoint[r][c][1]], color='r', linestyle='dashed')

        fig.suptitle('Number of tunnels: ' + str(len(tunnels)))
        axs[r,c].set_title('SP=' + str(routeStartPoint[r][c]) + ' | EP=' + str(routeEndPoint[r][c]) + ' | AF=' + str(allowFlipping[r][c]) + ' | l=' + str(totalLength) + '@' + str(timeDelta) + 'ms', fontsize=10)
        axs[r,c].set_xlim([-5, 105])
        axs[r,c].set_ylim([-5, 105])
        axs[r,c].set_aspect('equal')

plt.show()
