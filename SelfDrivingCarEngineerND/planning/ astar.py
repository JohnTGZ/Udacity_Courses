from queue import PriorityQueue

from regex import S

def breadthFirstSearch(s_start, s_goal):
    '''
    Also known as floodfill
    '''
    parent[s_start] = s_start

    openlist = PriorityQueue()
    openlist.insert(start)

    closedlist = set()

    while openlist:
        s = openlist.pop()
        closedlist.insert(S)
        for s_nb in getNeighbours(s):
            if s_nb not in closedlist:
                if s_nb not in openlist:
                    parent[s_nb] = s
                    openlist.add(s_nb)
                
def djikstra(s_start, s_goal):
    pass

def greedyBestFirstSearch(s_start, s_goal):
    pass


def astar(s_start, s_goal):
    g_cost[s_start] = 0
    parent[s_start] = s_start
    
    #Openlist is a priority queue that keeps track of all the cells to visit
    openlist = PriorityQueue()
    openlist.insert(s_start, g_cost[s_start] + h_cost[s_start])

    #Closed list keeps track of visited cells
    closedlist = set()
    
    while openlist: 
        s = openlist.pop()
        if s == s_goal:
            return "path found"
        closedlist.push(s) #set s as visited
        for s_nb in getNeighbours(s):
            if s_nb not in closedlist: #Not yet visited
                if s_nb not in openlist: #Not yet considered for visiting (Prevents us from considering cells already added to openlist)
                    g_cost[s_nb] = INF
                    parent[s_nb] = NULL
                    updateVertex(s, s_nb)

    return "No Path Found"


def updateVertex(s, s_nb):
    #If the current cost of 
    if (g_cost[s] + cost(s, s_nb) < g_cost[s_nb]):
        g_cost[s_nb] = g_cost[s] + cost(s, s_nb)
        parent[s_nb] = s
        if s_nb in openlist:
            openlist.remove(s_nb)
        openlist.insert(s_nb, g_cost[s_nb] + h_cost[s_nb])


#Hybrid AStar

def expand(state, goal):
    '''
    State(x, y, theta, g, f): An object which stores x, y coordinates, direction theta, and current g and f values.
    
    
    '''
    next_states = []

    for delta in range(-35, 40, 5):
        #Create a trajectory with delta as the steering angel using the
        #Bicycle model

        #Bicycle model
        delta_rad = deg_to_rad(delta):
        omega = SPEED/LENGTH * tan(delta_rad)
        next_x = state.x + SPEED * cos(theta)
        next_y = state.y + SPEED * sin(theta)
        next_theta = normalize(state.theta + omega)

        next_g = state.g + 1
        next_f = next_g + heuristic(next_x, next_y, goal)

        state = State(next_x, next_y, next_theta, next_g, next_f)
        next_states.append(state)
    
    return next_states

def search(grid, start, goal):
    # The opened array keeps track of the stack of States object we are searching
    # through
    opened = []

    # 3D array of zeros with dimensions:
    # (NUM_THETA_CELLS, grid x size, grid y size)
    closed = [[[0 for x in range(grid[0])] for y in range(len(grid))] 
        for cell in range(NUM_THETA_CELLS)]

    # 3D array with same dimensions. Will be filled with State() objects to 
    # keep track of the path through the grid.
    came_from = [[[0 for x in range(grid[0])] for y in range(len(grid))] 
        for cell in range(NUM_THETA_CELLS)]

    #Create new state object to start the search with
    x = start.x
    y = start.y
    theta = start.theta
    g = 0
    f = heuristic(start.x, start.y, goal)
    state = State(x, y, theta, 0, f)
    opened.append(state)

    # The range from 0 to 2pi has been discretized into NUM_THETA_CELLS cells. 
    # Here, theta_to_stack_number returns the cell that theta belongs to. 
    # Smaller thetas (close to 0 when normalized  into the range from 0 to 
    # 2pi) have lower stack numbers, and larger thetas (close to 2pi when 
    # normalized) have larger stack numbers.
    stack_num = theta_to_stack_number(state.theta)
    closed[stack_num][index(state.x)][index(state.y)] = 1


    # Store our starting state. For other states, we will store the previous 
    # state in the path, but the starting state has no previous.
    came_from[stack_num][index(state.x)][index(state.y)] = state

    while opened:
        # Sort the values by f-value and start search using the state with 
        # the lowest f-value. 
        opened.sort(key = lambda state:state.f)
        current = opened.pop(0)

        # Check if the x and y coordinates are in the same grid cell 
        # as the goal. (Note: The idx function returns the grid index for 
        # a given coordinate.)
        if (idx(current.x) == goal[0]) and (idx(current.y) == goal.y):
            # If so, the trajectory has reached the goal.
            return path

        # Otherwise, expand the current state to get a list of possible 
        # next states.
        next_states = expand(current, goal)
        for next_s in next_states:
            #If we have expanded outside the grid, skip this next_s:
            if next_s is not in the grid
                continue
            # Otherwise, check that we haven't already visited this cell and
            # that there is not an obstacle in the grid there.
            stack_num = theta_to_stack_number(next_s.theta)
            if closed[stack_num][idx(next_s.x)][idx(next_s.y)] == 0 
                and grid[idx(next_s.x)][idx(next_s.y)] == 0:
                # The state can be added to the opened stack.
                opened.append(next_s)
                # The stack_number, idx(next_s.x), idx(next_s.y) tuple 
                # has now been visited, so it can be closed.
                closed[stack_num][idx(next_s.x)][idx(next_s.y)] = 1
                # The next_s came from the current state, and is recorded.
                came_from[stack_num][idx(next_s.x)][idx(next_s.y)] = current