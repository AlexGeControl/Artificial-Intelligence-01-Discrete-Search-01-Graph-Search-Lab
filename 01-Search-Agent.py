
# coding: utf-8

# # ColumbiaX-01-Search-Agent

# ## Set Up Session

# In[1]:

# System:
import sys
# Board dimension:
from math import sqrt
# Data structures:
from collections import namedtuple
from collections import deque
import heapq
# Performance stats:
import time
import resource


# ## Parse Parameters

# In[2]:

# Parse method:
board = "1,2,5,3,4,0,6,7,8"


# In[3]:

# Parse init state:
init_state = tuple(
    int(x) for x in board.split(",")
)


# In[4]:

# Set board dimension & goal state:
N = int(sqrt(max(init_state)+1))
GOAL_STATE = tuple(range(N**2))


# ## Utilities

# In[5]:

# N-puzzle state:
NPuzzleState = namedtuple(
    'NPuzzleState', 
    'heuristic_cost, path_to_state, state, cost_of_path, search_depth'
)


# In[6]:

# BFS frontier:
class BFSFrontier:
    def __init__(self):
        self._frontier = deque()
        self._state_set = set()
        
    def put(self, state):
        self._frontier.appendleft(state)
        self._state_set.add(state.state)
    
    def get(self):
        state = self._frontier.pop()
        self._state_set.remove(state.state)
        return state
    
    def __contains__(self, state):
        return state.state in self._state_set
    
    def __len__(self):
        return len(self._frontier)
    
    def is_empty(self):
        return False if self._frontier else True


# In[7]:

# DFS frontier:
class DFSFrontier:
    def __init__(self):
        self._frontier = deque()
        self._state_set = set()
        
    def put(self, state):
        self._frontier.append(state)
        self._state_set.add(state.state)
        
    def get(self):
        state = self._frontier.pop()
        self._state_set.remove(state.state)
        return state
    
    def __contains__(self, state):
        return state.state in self._state_set
    
    def __len__(self):
        return len(self._frontier)
    
    def is_empty(self):
        return False if self._frontier else True


# In[8]:

# A-star frontier:
class AStarFrontier:
    def __init__(self):
        self._frontier = []
        self._state_set = set()
    
    def put(self, state):
        heapq.heappush(self._frontier, state)
        self._state_set.add(state.state)
        
    def get(self):
        state = heapq.heappop(self._frontier)
        self._state_set.remove(state.state)
        return state
    
    def update(self, state):
        idx = next(i for i,s in enumerate(self._frontier) if s.state == state.state)
        if (state.heuristic_cost < self._frontier[idx].heuristic_cost):
            self._frontier[idx] = self._frontier[-1]
            self._frontier.pop()
            heapq.heapify(self._frontier)
            heapq.heappush(self._frontier, state)
        
    def __contains__(self, state):
        return state.state in self._state_set
    
    def __len__(self):
        return len(self._frontier)
    
    def is_empty(self):
        return len(self._frontier) == 0


# In[9]:

def manhattan_distance(board):
    """ Total Manhattan distance between current board and goal board
    
    """
    total_manhattan_distance = 0
    for cur_board_idx, goal_board_idx in enumerate(board):
        if goal_board_idx != 0:
            cur_row_idx, cur_col_idx = cur_board_idx//N, cur_board_idx%N
            goal_row_idx, goal_col_idx = goal_board_idx//N, goal_board_idx%N
            total_manhattan_distance += abs(goal_row_idx-cur_row_idx) + abs(goal_col_idx-cur_col_idx)
    return total_manhattan_distance


# In[10]:

def is_goal_state(state):
    """ Whether current state is the goal state
    """
    return state.state == GOAL_STATE


# In[11]:

def generate_neighbors(n_puzzle_state, distance=None):
    """ Generate neighbors for the given n puzzle state
    
    """
    # Find the index of blank space:
    blank_idx = n_puzzle_state.state.index(0)
    row_idx, col_idx = blank_idx//N, blank_idx%N 
    # Move sequence--in 'Up','Down','Left','Right' order:
    moves = [
        (0, -1, 0),
        (1, 1, 0),
        (2, 0, -1),
        (3, 0, 1)
    ]
    # Remove illegal moves:
    if row_idx == 0:
        del moves[0]
    elif row_idx == (N - 1):
        del moves[1]
    if col_idx == 0:
        del moves[-2]
    elif col_idx == (N - 1):
        del moves[-1]
    # Generate neighbors:
    for move, row_delta, col_delta in moves:
        # New state:
        new_state = list(n_puzzle_state.state)
        new_blank_idx = (row_idx+row_delta)*N+(col_idx+col_delta)
        new_state[blank_idx], new_state[new_blank_idx] = new_state[new_blank_idx], new_state[blank_idx]
        new_state = tuple(new_state)
        # New path:
        new_path_to_state = n_puzzle_state.path_to_state + (move, )
        # New path cost:
        new_cost_of_path = n_puzzle_state.cost_of_path + 1
        # New depth:
        new_search_depth = n_puzzle_state.search_depth + 1
        
        yield NPuzzleState(
            heuristic_cost = new_cost_of_path + distance(new_state) if (not distance is None) else 0,
            path_to_state = new_path_to_state,
            state = tuple(new_state),
            cost_of_path = new_cost_of_path,
            search_depth = new_search_depth            
        )


# In[12]:

def generate_report(goal_state, goal_stats):
    """
    """
    # Move literals:
    moves = ['Up', 'Down', 'Left', 'Right']
    # Report:
    report = """
    path_to_goal: {}
    cost_of_path: {}
    nodes_expanded: {}
    fringe_size: {}
    max_fringe_size: {}
    search_depth: {}
    max_search_depth: {}
    running_time: {:.8f}
    max_ram_usage: {:.8f}
    """.format(
        list(moves[idx] for idx in goal_state.path_to_state) if not goal_state is None else None,
        goal_state.cost_of_path if not goal_state is None else None,
        goal_stats['nodes_expanded'] - 1,
        goal_stats['fringe_size'],
        goal_stats['max_fringe_size'],
        goal_state.search_depth if not goal_state is None else None,
        goal_stats['max_search_depth'],
        goal_stats['running_time'],
        goal_stats['max_ram_usage']        
    )
    return report


# ## Initialize Problem

# In[13]:

# Init root:
root = NPuzzleState(
    heuristic_cost = 0 + manhattan_distance(init_state),
    path_to_state = (),
    state = init_state,
    cost_of_path = 0,
    search_depth = 0
)
# Init search stats
stats = dict(
    nodes_expanded = 0, 
    fringe_size = 0, 
    max_fringe_size = 0, 
    max_search_depth = 0, 
    running_time = 0.0, 
    max_ram_usage = 0.0
)


# ## BFS

# In[14]:

def bfs_solve(root, stats):
    # Init frontier & explored set:
    frontier = BFSFrontier()
    frontier.put(root)
    explored = set()
    goal_state = None
    
    # Start timer:
    start_time = time.time()
    
    # Solve:
    while not frontier.is_empty():
        state = frontier.get()
        explored.add(state.state)
        
        # Stats: nodes expanded:
        stats['nodes_expanded'] += 1                
        
        if is_goal_state(state):
            # Stats: fringe size:
            stats['fringe_size'] = len(frontier)
            goal_state = state
            break

        for neighbor in generate_neighbors(state):
            if (neighbor not in frontier) and (neighbor.state not in explored):
                # Stats: max search depth:
                if neighbor.search_depth > stats['max_search_depth']:
                    stats['max_search_depth'] = neighbor.search_depth
                # Add to frontier:
                frontier.put(neighbor)
        
        # Stats: max fringe size:
        if len(frontier) > stats['max_fringe_size']:
            stats['max_fringe_size'] = len(frontier)
        # Stats: max ram usage:
        ram_usage = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1000
        if ram_usage > stats['max_ram_usage']:
            stats['max_ram_usage'] = ram_usage
    
    stats['running_time'] = time.time() - start_time
    
    return (goal_state, stats)


# In[15]:

# Solution:
stats = dict(
    nodes_expanded = 0, 
    fringe_size = 0, 
    max_fringe_size = 0, 
    max_search_depth = 0, 
    running_time = 0.0, 
    max_ram_usage = 0.0
)
goal_bfs, goal_stats = bfs_solve(root, stats)


# In[16]:

# Report:
print(
    generate_report(goal_bfs, goal_stats)
)


# ## DFS:

# In[17]:

def dfs_solve(root, stats):
    # Init frontier & explored set:
    frontier = DFSFrontier()
    frontier.put(root)
    explored = set()
    goal_state = None
    
    # Start timer:
    start_time = time.time()    
    
    # Solve:
    while not frontier.is_empty():
        state = frontier.get()
        explored.add(state.state)
        # Stats: nodes expanded:
        stats['nodes_expanded'] += 1 
        
        if is_goal_state(state):
            # Stats: fringe size:
            stats['fringe_size'] = len(frontier)
            goal_state = state
            break

        for neighbor in reversed(
            list(
                generate_neighbors(state)
            )
        ):
            if (neighbor not in frontier) and (neighbor.state not in explored):
                # Stats: max search depth:
                if neighbor.search_depth > stats['max_search_depth']:
                    stats['max_search_depth'] = neighbor.search_depth
                frontier.put(neighbor)
                
                
        # Stats: max fringe size:
        if len(frontier) > stats['max_fringe_size']:
            stats['max_fringe_size'] = len(frontier)
        # Stats: max ram usage:
        ram_usage = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1000
        if ram_usage > stats['max_ram_usage']:
            stats['max_ram_usage'] = ram_usage
    
    stats['running_time'] = time.time() - start_time
    
    return (goal_state, stats)


# In[18]:

# Solution:
stats = dict(
    nodes_expanded = 0, 
    fringe_size = 0, 
    max_fringe_size = 0, 
    max_search_depth = 0, 
    running_time = 0.0, 
    max_ram_usage = 0.0
)
# goal_dfs, goal_stats = dfs_solve(root, stats)


# In[19]:

# Report:
# print(
#    generate_report(goal_dfs, goal_stats)
#)


# ## A-Star

# In[20]:

def a_star_solve(root, stats):
    # Init frontier & explored set:
    frontier = AStarFrontier()
    frontier.put(root)
    explored = set()
    goal_state = None
    
    # Start timer:
    start_time = time.time()
    
    # Solve:
    while not frontier.is_empty():
        state = frontier.get()
        explored.add(state.state)
        
        # Stats: nodes expanded:
        stats['nodes_expanded'] += 1                
        
        if is_goal_state(state):
            # Stats: fringe size:
            stats['fringe_size'] = len(frontier)
            goal_state = state
            break

        for neighbor in generate_neighbors(state, manhattan_distance):
            if (neighbor not in frontier) and (neighbor.state not in explored):
                # Stats: max search depth:
                if neighbor.search_depth > stats['max_search_depth']:
                    stats['max_search_depth'] = neighbor.search_depth
                # Add to frontier:
                frontier.put(neighbor)
            elif (neighbor in frontier):
                frontier.update(neighbor)
        
        # Stats: max fringe size:
        if len(frontier) > stats['max_fringe_size']:
            stats['max_fringe_size'] = len(frontier)
        # Stats: max ram usage:
        ram_usage = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1000
        if ram_usage > stats['max_ram_usage']:
            stats['max_ram_usage'] = ram_usage
    
    stats['running_time'] = time.time() - start_time
    
    return (goal_state, stats)


# In[21]:

# Solution:
stats = dict(
    nodes_expanded = 0, 
    fringe_size = 0, 
    max_fringe_size = 0, 
    max_search_depth = 0, 
    running_time = 0.0, 
    max_ram_usage = 0.0
)
goal_a_star, goal_stats = a_star_solve(root, stats)


# In[22]:

# Report:
print(
    generate_report(goal_a_star, goal_stats)
)


# ## IDA-Star

# In[ ]:

def ida_star_solve(root, stats):
    def do_ida_star_solve(root, bound, stats):
        # Init frontier & explored set:
        frontier = DFSFrontier()
        frontier.put(root)
        explored = set()
        goal_state = None

        # Min cost:
        min_cost = 2**(sys.maxsize-1)

        # Start timer:
        start_time = time.time()    

        # Solve:
        while not frontier.is_empty():
            state = frontier.get()
            explored.add(state.state)
            # Stats: nodes expanded:
            stats['nodes_expanded'] += 1 

            # If current node exceeds cost limit:
            if state.heuristic_cost > bound:
                min_cost = state.heuristic_cost
                break

            if is_goal_state(state):
                # Stats: fringe size:
                stats['fringe_size'] = len(frontier)
                goal_state = state
                break

            # Update min cost:
            if state.heuristic_cost < min_cost:
                min_cost = state.heuristic_cost

            for neighbor in reversed(
                list(
                    generate_neighbors(state, manhattan_distance)
                )
            ):
                if (neighbor not in frontier) and (neighbor.state not in explored):
                    # Stats: max search depth:
                    if neighbor.search_depth > stats['max_search_depth']:
                        stats['max_search_depth'] = neighbor.search_depth
                    frontier.put(neighbor)


            # Stats: max fringe size:
            if len(frontier) > stats['max_fringe_size']:
                stats['max_fringe_size'] = len(frontier)
            # Stats: max ram usage:
            ram_usage = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1000
            if ram_usage > stats['max_ram_usage']:
                stats['max_ram_usage'] = ram_usage

        stats['running_time'] = time.time() - start_time

        return (goal_state, min_cost, stats)
    
    # Init:
    bound = root.heuristic_cost
    goal_state = None
    
    # Iteratively deepen:
    while goal_state is None:
        goal_state, bound, goal_stats = do_ida_star_solve(
            root, 
            dict(
                nodes_expanded = 0, 
                fringe_size = 0, 
                max_fringe_size = 0, 
                max_search_depth = 0, 
                running_time = 0.0, 
                max_ram_usage = 0.0
            ), 
            bound
        )
    
    return (goal_state, goal_stats)


# In[ ]:

# Solution:
stats = dict(
    nodes_expanded = 0, 
    fringe_size = 0, 
    max_fringe_size = 0, 
    max_search_depth = 0, 
    running_time = 0.0, 
    max_ram_usage = 0.0
)
goal_ida_star, goal_stats = ida_star_solve(root, stats)


# In[ ]:

# Report:
print(
    generate_report(goal_ida_star, goal_stats)
)

