import math
import random
import matplotlib.pyplot as plt
import time

def tsp_solver_tunnels(tunnels, allowFlipping=False, routeStartPoint=None, routeEndPoint=None):

    # STEP 1: Adds the routeStartPoint (it will be deleted at the end)
    if routeStartPoint is None:
        tunnels.insert(0,{'endX': 0, 'endY': 0})
    else:
        tunnels.insert(0,{'endX': routeStartPoint[0], 'endY': routeStartPoint[1]})

    # STEP 2: Applies nearest neighbour algorithm
    potentialNeighbours = tunnels[1:]
    route = [tunnels[0]]
    toBeFlipped = False
    while potentialNeighbours:
        costCurrent = float('inf')
        for neighbour in potentialNeighbours:
            costNew = (route[-1]['endX'] - neighbour['startX'])**2 + (route[-1]['endY'] - neighbour['startY'])**2
            if costNew < costCurrent:
                costCurrent = costNew
                toBeFlipped = False
                nearestNeighbour = neighbour
        if allowFlipping:
            for neighbour in potentialNeighbours:
                costNew = (route[-1]['endX'] - neighbour['endX'])**2 + (route[-1]['endY'] - neighbour['endY'])**2
                if costNew < costCurrent:
                    costCurrent = costNew
                    toBeFlipped = True
                    nearestNeighbour = neighbour
        potentialNeighbours.remove(nearestNeighbour)
        if toBeFlipped:
            nearestNeighbour['startX'], nearestNeighbour['endX'] = nearestNeighbour['endX'], nearestNeighbour['startX']
            nearestNeighbour['startY'], nearestNeighbour['endY'] = nearestNeighbour['endY'], nearestNeighbour['startY']
        route.append(nearestNeighbour)

    # STEP 3: Adds the routeEndPoint (it will be deleted at the end)
    if routeEndPoint is not None:
        route.append({'startX': routeEndPoint[0], 'startY': routeEndPoint[1]})

    # STEP 4: Additional improvement of the route
    limitReorderI = len(route) - 2
    if routeEndPoint is not None:
        limitReorderI -= 1
    limitReorderJ = len(route)
    limitFlipI = len(route) - 1
    limitRelocationI = len(route) - 1
    limitRelocationJ = len(route) - 1
    lastImprovementAtStep = 0 

    while True:

        if allowFlipping:
            # STEP 4.1: Applies 2-opt
            if lastImprovementAtStep == 1: break
            improvementFound = True
            while improvementFound:
                improvementFound = False
                for i in range(0,limitReorderI):
                    subRouteLengthCurrentPart = math.dist([route[i]['endX'],route[i]['endY']], [route[i+1]['startX'],route[i+1]['startY']])
                    for j in range(i+3,limitReorderJ):
                        subRouteLengthCurrent = subRouteLengthCurrentPart
                        subRouteLengthCurrent += math.dist([route[j-1]['endX'],route[j-1]['endY']], [route[j]['startX'],route[j]['startY']])
                        subRouteLengthNew = math.dist([route[i+1]['startX'],route[i+1]['startY']], [route[j]['startX'],route[j]['startY']])
                        subRouteLengthNew += math.dist([route[i]['endX'],route[i]['endY']], [route[j-1]['endX'],route[j-1]['endY']])
                        subRouteLengthNew += 1e-6
                        if subRouteLengthNew < subRouteLengthCurrent:
                            for k in range(i+1,j): # flips direction of each tunnel between i-th and j-th tunnel
                                route[k]['startX'], route[k]['endX'] = route[k]['endX'], route[k]['startX']
                                route[k]['startY'], route[k]['endY'] = route[k]['endY'], route[k]['startY']
                            route[i+1:j] = route[i+1:j][::-1] # reverse the order of tunnels between i-th and j-th tunnel
                            subRouteLengthCurrentPart = math.dist([route[i]['endX'],route[i]['endY']], [route[i+1]['startX'],route[i+1]['startY']])
                            improvementFound = True
                            lastImprovementAtStep = 1
                    if routeEndPoint is None:
                        subRouteLengthCurrent = math.dist([route[i]['endX'],route[i]['endY']], [route[i+1]['startX'],route[i+1]['startY']])
                        subRouteLengthNew = math.dist([route[i]['endX'],route[i]['endY']], [route[-1]['endX'],route[-1]['endY']])
                        subRouteLengthNew += 1e-6
                        if subRouteLengthNew < subRouteLengthCurrent:
                            for k in range(i+1,limitReorderJ): # flips direction of each tunnel after i-th to the last tunnel
                                route[k]['startX'], route[k]['endX'] = route[k]['endX'], route[k]['startX']
                                route[k]['startY'], route[k]['endY'] = route[k]['endY'], route[k]['startY']
                            route[i+1:limitReorderJ] = route[i+1:limitReorderJ][::-1] # reverse the order of tunnels after i-th to the last tunnel
                            improvementFound = True
                            lastImprovementAtStep = 1

            # STEP 4.2: Applies flipping
            if lastImprovementAtStep == 2: break
            improvementFound = True
            while improvementFound:
                improvementFound = False
                for i in range(1,limitFlipI):
                    subRouteLengthCurrent = math.dist([route[i-1]['endX'],route[i-1]['endY']], [route[i]['startX'],route[i]['startY']])
                    subRouteLengthCurrent += math.dist([route[i]['endX'],route[i]['endY']], [route[i+1]['startX'],route[i+1]['startY']])
                    subRouteLengthNew = math.dist([route[i-1]['endX'],route[i-1]['endY']], [route[i]['endX'],route[i]['endY']])
                    subRouteLengthNew += math.dist([route[i]['startX'],route[i]['startY']], [route[i+1]['startX'],route[i+1]['startY']])
                    subRouteLengthNew += 1e-6
                    if subRouteLengthNew < subRouteLengthCurrent:
                        # flips direction of i-th tunnel
                        route[i]['startX'], route[i]['endX'] = route[i]['endX'], route[i]['startX']
                        route[i]['startY'], route[i]['endY'] = route[i]['endY'], route[i]['startY']
                        improvementFound = True
                        lastImprovementAtStep = 2
                if routeEndPoint is None:
                    subRouteLengthCurrent = math.dist([route[-2]['endX'],route[-2]['endY']], [route[-1]['startX'],route[-1]['startY']])
                    subRouteLengthNew = math.dist([route[-2]['endX'],route[-2]['endY']], [route[-1]['endX'],route[-1]['endY']])
                    subRouteLengthNew += 1e-6
                    if subRouteLengthNew < subRouteLengthCurrent:
                        # flips direction of the last tunnel
                        route[-1]['startX'], route[-1]['endX'] = route[-1]['endX'], route[-1]['startX']
                        route[-1]['startY'], route[-1]['endY'] = route[-1]['endY'], route[-1]['startY']
                        improvementFound = True
                        lastImprovementAtStep = 2

        # STEP 4.3: Applies relocation
        if lastImprovementAtStep == 3: break
        improvementFound = True
        while improvementFound:
            improvementFound = False
            for i in range(1,limitRelocationI):
                subRouteLengthCurrentPart = math.dist([route[i-1]['endX'],route[i-1]['endY']], [route[i]['startX'],route[i]['startY']])
                subRouteLengthCurrentPart += math.dist([route[i]['endX'],route[i]['endY']], [route[i+1]['startX'],route[i+1]['startY']])
                subRouteLengthNewPart = math.dist([route[i-1]['endX'],route[i-1]['endY']], [route[i+1]['startX'],route[i+1]['startY']])
                subRouteLengthNewPart += 1e-6
                for j in range(0,i-2):
                    subRouteLengthCurrent = subRouteLengthCurrentPart
                    subRouteLengthCurrent += math.dist([route[j]['endX'],route[j]['endY']], [route[j+1]['startX'],route[j+1]['startY']])
                    subRouteLengthNew = subRouteLengthNewPart
                    subRouteLengthNew += math.dist([route[j]['endX'],route[j]['endY']], [route[i]['startX'],route[i]['startY']])
                    subRouteLengthNew += math.dist([route[i]['endX'],route[i]['endY']], [route[j+1]['startX'],route[j+1]['startY']])
                    if subRouteLengthNew < subRouteLengthCurrent:
                        route.insert(j+1, route.pop(i)) # relocate the i-th tunnel backward (after j-th tunnel)
                        subRouteLengthCurrentPart = math.dist([route[i-1]['endX'],route[i-1]['endY']], [route[i]['startX'],route[i]['startY']])
                        subRouteLengthCurrentPart += math.dist([route[i]['endX'],route[i]['endY']], [route[i+1]['startX'],route[i+1]['startY']])
                        subRouteLengthNewPart = math.dist([route[i-1]['endX'],route[i-1]['endY']], [route[i+1]['startX'],route[i+1]['startY']])
                        subRouteLengthNewPart += 1e-6
                        improvementFound = True
                        lastImprovementAtStep = 3
                for j in range(i+1,limitRelocationJ):
                    subRouteLengthCurrent = subRouteLengthCurrentPart
                    subRouteLengthCurrent += math.dist([route[j]['endX'],route[j]['endY']], [route[j+1]['startX'],route[j+1]['startY']])
                    subRouteLengthNew = subRouteLengthNewPart
                    subRouteLengthNew += math.dist([route[j]['endX'],route[j]['endY']], [route[i]['startX'],route[i]['startY']])
                    subRouteLengthNew += math.dist([route[i]['endX'],route[i]['endY']], [route[j+1]['startX'],route[j+1]['startY']])
                    if subRouteLengthNew < subRouteLengthCurrent:
                        route.insert(j, route.pop(i)) # relocate the i-th tunnel forward (after j-th tunnel)
                        subRouteLengthCurrentPart = math.dist([route[i-1]['endX'],route[i-1]['endY']], [route[i]['startX'],route[i]['startY']])
                        subRouteLengthCurrentPart += math.dist([route[i]['endX'],route[i]['endY']], [route[i+1]['startX'],route[i+1]['startY']])
                        subRouteLengthNewPart = math.dist([route[i-1]['endX'],route[i-1]['endY']], [route[i+1]['startX'],route[i+1]['startY']])
                        subRouteLengthNewPart += 1e-6
                        improvementFound = True
                        lastImprovementAtStep = 3
            if routeEndPoint is None:
                subRouteLengthCurrentPart = math.dist([route[-2]['endX'],route[-2]['endY']], [route[-1]['startX'],route[-1]['startY']])
                for j in range(0,len(route)-2):
                    subRouteLengthCurrent = subRouteLengthCurrentPart
                    subRouteLengthCurrent += math.dist([route[j]['endX'],route[j]['endY']], [route[j+1]['startX'],route[j+1]['startY']])
                    subRouteLengthNew = math.dist([route[j]['endX'],route[j]['endY']], [route[-1]['startX'],route[-1]['startY']])
                    subRouteLengthNew += math.dist([route[-1]['endX'],route[-1]['endY']], [route[j+1]['startX'],route[j+1]['startY']])
                    subRouteLengthNew += 1e-6
                    if subRouteLengthNew < subRouteLengthCurrent:
                        route.insert(j+1, route.pop(-1)) # relocate the last tunnel after j-th tunnel
                        subRouteLengthCurrentPart = math.dist([route[-2]['endX'],route[-2]['endY']], [route[-1]['startX'],route[-1]['startY']])
                        improvementFound = True
                        lastImprovementAtStep = 3

        if lastImprovementAtStep == 0: break # no additional improvementes could be made

    # STEP 5: Deletes temporary start and end point
    del route[0]
    if routeEndPoint is not None:
        del route[-1]

    return route


# --- code below this line is just to demonstrate the capability of tsp_solver_tunnels() function ---


# STEP 1: Generate list of tunnels (1=generate ; 0=don't generate)
genTunnelsHori = 1 # 100 tunnels, horizontal pattern
genTunnelsRand = 0 # 20 tunnels, random position

tunnels = []

if genTunnelsHori:
    for i in range(10):
        for j in range(10):
            tunnels.append({'startX': 10*i+5, 'startY': 10*j+5, 'endX': 10*i+10, 'endY': 10*j+5})

if genTunnelsRand:
    for i in range(20):
        tunnels.append({'startX': 100*random.random(), 'startY': 100*random.random(), 'endX': 100*random.random(), 'endY': 100*random.random()})

# STEP 2: Define allowFlipping, start and end point for each example
allowFlipping = True

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

        # shuffle the list of tunnels before sorting
        tunnels = random.sample(tunnels, len(tunnels))

        # run the solver and measure the time needed
        print('Solving row ' + str(r+1) + ', column ' + str(c+1) + '...')
        timeStart = time.time()
        tunnels = tsp_solver_tunnels(tunnels, allowFlipping, routeStartPoint[r][c], routeEndPoint[r][c])
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
            axs[r,c].plot([tunnels[i]['startX'], tunnels[i]['endX']], [tunnels[i]['startY'], tunnels[i]['endY']], linewidth=1, color='b')

        # draw route
        for i in range(len(tunnels)-1):
            axs[r,c].plot([tunnels[i]['endX'], tunnels[i+1]['startX']], [tunnels[i]['endY'], tunnels[i+1]['startY']], color='r')

        # draw path from routeStartPoint
        axs[r,c].scatter(routeStartPoint[r][c][0], routeStartPoint[r][c][1], color='b', marker='>')
        axs[r,c].plot([routeStartPoint[r][c][0], tunnels[0]['startX']], [routeStartPoint[r][c][1], tunnels[0]['startY']], color='r', linestyle='dashed')

        # draw path to routeEndPoint
        if routeEndPoint[r][c] is not None:
            axs[r,c].scatter(routeEndPoint[r][c][0], routeEndPoint[r][c][1], color='b', marker='s')
            axs[r,c].plot([tunnels[-1]['endX'], routeEndPoint[r][c][0]], [tunnels[-1]['endY'], routeEndPoint[r][c][1]], color='r', linestyle='dashed')

        fig.suptitle('Number of tunnels: ' + str(len(tunnels)) + ' | allowFlipping: ' + str(allowFlipping))
        axs[r,c].set_title('SP=' + str(routeStartPoint[r][c]) + ' | EP=' +str(routeEndPoint[r][c]) + ' | t=' + str(timeDelta) + 'ms' + ' | l=' + str(totalLength), fontsize=10)
        axs[r,c].set_xlim([-5, 105])
        axs[r,c].set_ylim([-5, 105])
        axs[r,c].set_aspect('equal')

plt.show()
