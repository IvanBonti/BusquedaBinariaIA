import pygame
import heapq

# Definiciones
WINDOW_SIZE = (600, 600)
GRID_SIZE = (30, 30)
CELL_SIZE = WINDOW_SIZE[0] // GRID_SIZE[0]
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)

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
    # Distancia Manhattan
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
        neighbors = [(x-1, y), (x+1, y), (x, y-1), (x, y+1), (x-1, y-1), (x+1, y+1), (x-1, y+1), (x+1, y-1)]
        
        for next_pos in neighbors:
            if (0 <= next_pos[0] < len(grid) and
                0 <= next_pos[1] < len(grid[0]) and
                grid[next_pos[0]][next_pos[1]] != 1):
                
                if next_pos in closed_list:
                    continue
                
                g = current_node.g + (1.414 if (next_pos[0] != x and next_pos[1] != y) else 1)
                h = heuristic(next_pos, goal_node.position)
                neighbor_node = Node(next_pos, g, h, current_node)
                
                if all(neighbor_node.f < node.f for node in open_list if node.position == next_pos):
                    heapq.heappush(open_list, neighbor_node)
    
    return None

def draw_grid(win, grid, path=None):
    for x in range(len(grid)):
        for y in range(len(grid[0])):
            color = WHITE if grid[x][y] == 0 else BLACK
            pygame.draw.rect(win, color, pygame.Rect(y * CELL_SIZE, x * CELL_SIZE, CELL_SIZE, CELL_SIZE))
    if path:
        for (x, y) in path:
            pygame.draw.rect(win, GREEN, pygame.Rect(y * CELL_SIZE, x * CELL_SIZE, CELL_SIZE, CELL_SIZE))

def main():
    pygame.init()
    win = pygame.display.set_mode(WINDOW_SIZE)
    pygame.display.set_caption('A* Pathfinding Visualization')

    # Crear una cuadrícula con obstáculos
    grid = [[0 for _ in range(GRID_SIZE[1])] for _ in range(GRID_SIZE[0])]
    for i in range(10, 20):
        for j in range(GRID_SIZE[1]):
            grid[i][j] = 1
    grid[15][15] = 0
    grid[16][15] = 1

    start = (0, 0)
    goal = (29, 29)

    path = astar(grid, start, goal)

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        win.fill(BLACK)
        draw_grid(win, grid, path)
        pygame.display.update()

    pygame.quit()

if __name__ == '__main__':
    main()
