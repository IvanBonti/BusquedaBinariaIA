import heapq

class Node:
    def __init__(self, position, g=0, h=0, parent=None):
        self.position = position
        self.g = g
        self.h = h
        self.f = g + h
        self.parent = parent

    def __lt__(self, other):
        return self.f < other.f

def heuristic(a, b):
    # Heurística de distancia Manhattan
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def astar(grid, start, goal):
    open_list = []
    closed_list = set()
    
    start_node = Node(start, 0, heuristic(start, goal))
    goal_node = Node(goal)
    
    heapq.heappush(open_list, start_node)
    
    while open_list:
        current_node = heapq.heappop(open_list)
        closed_list.add(current_node.position)
        
        if current_node.position == goal_node.position:
            path = []
            while current_node:
                path.append(current_node.position)
                current_node = current_node.parent
            return path[::-1]
        
        (x, y) = current_node.position
        neighbors = [(x-1, y), (x+1, y), (x, y-1), (x, y+1)]
        
        for next_pos in neighbors:
            if (0 <= next_pos[0] < len(grid) and
                0 <= next_pos[1] < len(grid[0]) and
                grid[next_pos[0]][next_pos[1]] != 1):
                
                if next_pos in closed_list:
                    continue
                
                g = current_node.g + 1
                h = heuristic(next_pos, goal_node.position)
                neighbor_node = Node(next_pos, g, h, current_node)
                
                if all(neighbor_node.f < node.f for node in open_list if node.position == next_pos):
                    heapq.heappush(open_list, neighbor_node)
    
    return None

# Ejemplo de cuadrícula (0 = libre, 1 = obstáculo)
grid = [
    [0, 0, 0, 0, 0],
    [0, 1, 1, 1, 0],
    [0, 0, 0, 0, 0],
    [0, 0, 1, 1, 0],
    [0, 0, 0, 0, 0]
]

start = (0, 0)
goal = (4, 4)

path = astar(grid, start, goal)
print("Path:", path)
