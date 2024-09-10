import heapq

# Clase para representar un nodo en la red
class Node:
    def __init__(self, name, g=0, h=0, parent=None):
        self.name = name
        self.g = g  # Costo acumulado (g)
        self.h = h  # Heurística estimada (h)
        self.f = g + h  # Costo total (f = g + h)
        self.parent = parent  # Nodo anterior en la ruta

    def __lt__(self, other):
        return self.f < other.f

# Heurística basada en la latencia mínima entre dos nodos
def heuristic(node1, node2, latency_matrix):
    try:
        return latency_matrix[node1][node2]
    except KeyError:
        # Si no existe una latencia definida, devolver un valor grande
        return float('inf')


# Algoritmo A* para optimización de enrutamiento
def astar_optimized(network, start, goal, bandwidth_matrix, latency_matrix):
    open_list = []
    closed_list = set()

    # Nodo inicial
    start_node = Node(start, 0, heuristic(start, goal, latency_matrix))
    heapq.heappush(open_list, start_node)

    while open_list:
        current_node = heapq.heappop(open_list)
        closed_list.add(current_node.name)

        # Si alcanzamos el nodo objetivo, reconstruimos la ruta
        if current_node.name == goal:
            path = []
            total_cost = current_node.f  # Guardar el costo antes de perder el nodo
            while current_node:
                path.append(current_node.name)
                current_node = current_node.parent
            return path[::-1], total_cost
          # Ruta inversa y costo total

        # Explorar vecinos del nodo actual
        for neighbor in network[current_node.name]:
            if neighbor in closed_list:
                continue

            bandwidth = bandwidth_matrix[current_node.name][neighbor]
            if bandwidth <= 0:  # Si no hay ancho de banda disponible
                continue

            latency = latency_matrix[current_node.name][neighbor]
            transmission_time = 1 / bandwidth  # Inversamente proporcional al ancho de banda

            # Costo acumulado = tiempo de transmisión + latencia
            g = current_node.g + transmission_time + latency
            h = heuristic(neighbor, goal, latency_matrix)  # Estimación heurística
            neighbor_node = Node(neighbor, g, h, parent=current_node)

            # Añadir nodo a la lista abierta si es la mejor ruta hacia el vecino
            if all(neighbor_node.f < node.f for node in open_list if node.name == neighbor):
                heapq.heappush(open_list, neighbor_node)

    return None, float('inf')  # Si no se encuentra ruta

# Ejemplo de red con ancho de banda y latencia
def main():
    # Grafo de la red (conexiones entre nodos)
    network = {
        'A': ['B', 'C', 'D'],
        'B': ['A', 'E', 'F'],
        'C': ['A', 'F', 'G'],
        'D': ['A', 'G'],
        'E': ['B', 'H'],
        'F': ['B', 'C', 'I'],
        'G': ['C', 'D', 'I'],
        'H': ['E', 'I', 'J'],
        'I': ['F', 'G', 'H', 'J'],
        'J': ['H', 'I']
    }

    # Matriz de ancho de banda (Mbps)
    bandwidth_matrix = {
        'A': {'B': 100, 'C': 80, 'D': 50},
        'B': {'A': 100, 'E': 90, 'F': 60},
        'C': {'A': 80, 'F': 120, 'G': 70},
        'D': {'A': 50, 'G': 40},
        'E': {'B': 90, 'H': 100},
        'F': {'B': 60, 'C': 120, 'I': 110},
        'G': {'C': 70, 'D': 40, 'I': 90},
        'H': {'E': 100, 'I': 80, 'J': 70},
        'I': {'F': 110, 'G': 90, 'H': 80, 'J': 150},
        'J': {'H': 70, 'I': 150}
    }

    # Matriz de latencia (milisegundos)
    latency_matrix = {
        'A': {'B': 10, 'C': 20, 'D': 30},
        'B': {'A': 10, 'E': 15, 'F': 25},
        'C': {'A': 20, 'F': 10, 'G': 15},
        'D': {'A': 30, 'G': 25},
        'E': {'B': 15, 'H': 20},
        'F': {'B': 25, 'C': 10, 'I': 5},
        'G': {'C': 15, 'D': 25, 'I': 30},
        'H': {'E': 20, 'I': 25, 'J': 10},
        'I': {'F': 5, 'G': 30, 'H': 25, 'J': 5},
        'J': {'H': 10, 'I': 5}
    }

    # Nodo de inicio y objetivo
    start = 'A'
    goal = 'J'

    # Ejecutar algoritmo A* optimizado
    path, cost = astar_optimized(network, start, goal, bandwidth_matrix, latency_matrix)

    if path:
        print(f"Ruta óptima encontrada: {' -> '.join(path)}")
    else:
        print("No se encontró una ruta válida")

if __name__ == "__main__":
    main()
