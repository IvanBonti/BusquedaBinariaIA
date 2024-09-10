import heapq

class Node:
    def __init__(self, position, g=0, h=0, traffic_factor=1, parent=None):
        self.position = position  # Coordenadas del nodo (x, y)
        self.g = g  # Costo desde el inicio hasta el nodo actual
        self.h = h  # Heurística estimada del nodo actual al objetivo
        self.traffic_factor = traffic_factor  # Factor de tráfico que afecta el costo
        self.f = g + h  # Costo total f = g + h
        self.parent = parent  # Nodo padre en la ruta

    def __lt__(self, other):
        return self.f < other.f  # Compara el nodo basado en f-cost

# Heurística: Distancia Manhattan
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

# Algoritmo A* para buscar la ruta más corta
def astar(grid, start, goal, traffic):
    open_list = []  # Lista de nodos por explorar
    closed_list = set()  # Conjunto de nodos ya explorados
    start_node = Node(start, 0, heuristic(start, goal))
    heapq.heappush(open_list, start_node)

    while open_list:
        current_node = heapq.heappop(open_list)
        closed_list.add(current_node.position)

        # Si llegamos al objetivo, construimos la ruta
        if current_node.position == goal:
            path = []
            while current_node:
                path.append(current_node.position)
                current_node = current_node.parent
            return path[::-1]  # Devolver la ruta desde inicio a objetivo

        # Generar vecinos
        (x, y) = current_node.position
        neighbors = [(x-1, y), (x+1, y), (x, y-1), (x, y+1)]  # Movimientos (N, S, E, O)

        for next_pos in neighbors:
            if (0 <= next_pos[0] < len(grid) and 0 <= next_pos[1] < len(grid[0]) and grid[next_pos[0]][next_pos[1]] != 1):
                if next_pos in closed_list:
                    continue

                # Calcular el coste g para el vecino
                g = current_node.g + 1 * traffic[next_pos[0]][next_pos[1]]  # Incluye factor de tráfico
                h = heuristic(next_pos, goal)
                neighbor_node = Node(next_pos, g, h, traffic_factor=traffic[next_pos[0]][next_pos[1]], parent=current_node)

                # Agregar a la lista abierta si no está en ella o si encontramos un mejor camino
                if all(neighbor_node.f < node.f for node in open_list if node.position == next_pos):
                    heapq.heappush(open_list, neighbor_node)

    return None  # No se encontró ruta

# Mostrar la cuadrícula y la ruta
def print_grid(grid, path=None):
    for x in range(len(grid)):
        for y in range(len(grid[0])):
            if path and (x, y) in path:
                print("P", end=" ")  # Ruta
            elif grid[x][y] == 1:
                print("#", end=" ")  # Obstáculo
            else:
                print(".", end=" ")  # Espacio vacío
        print()

def main():
    # Definir una cuadrícula (0 = espacio libre, 1 = obstáculo)
    grid = [
        [0, 0, 0, 0, 1, 0],
        [0, 1, 0, 0, 1, 0],
        [0, 1, 0, 1, 0, 0],
        [0, 0, 0, 1, 0, 0],
        [0, 1, 1, 1, 0, 1],
        [0, 0, 0, 0, 0, 0]
    ]

    # Factor de tráfico en cada celda (1 = sin tráfico, >1 = tráfico pesado)
    traffic = [
        [1, 1, 1, 1, 1, 1],
        [1, 2, 1, 1, 1, 1],
        [1, 2, 1, 2, 1, 1],
        [1, 1, 1, 2, 1, 1],
        [1, 2, 2, 2, 1, 2],
        [1, 1, 1, 1, 1, 1]
    ]

    start = (0, 0)  # Punto de inicio
    goal = (5, 5)   # Punto de destino

    # Buscar la ruta
    path = astar(grid, start, goal, traffic)
    
    if path:
        print("Ruta encontrada:")
        print_grid(grid, path)
    else:
        print("No se encontró una ruta.")

if __name__ == '__main__':
    main()
