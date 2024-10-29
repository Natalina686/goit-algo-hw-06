import networkx as nx
import matplotlib.pyplot as plt
import heapq  # Перемістіть імпорт сюди

# Завдання 1: Створення та візуалізація графа
G = nx.Graph()

# Додавання вершин (зупинок)
stops = ['A', 'B', 'C', 'D', 'E']
G.add_nodes_from(stops)

# Додавання ребер (доріг) з вагами
edges = [('A', 'B', 1), ('A', 'C', 4), ('B', 'C', 2), ('B', 'D', 5), ('C', 'D', 1), ('D', 'E', 3)]
G.add_weighted_edges_from(edges)

# Візуалізація графа
pos = nx.spring_layout(G)
nx.draw(G, pos, with_labels=True, node_color='lightblue', node_size=1000, font_size=16)
edge_labels = nx.get_edge_attributes(G, 'weight')
nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels)
plt.title("Транспортна мережа")
plt.show()

# Аналіз характеристик
num_nodes = G.number_of_nodes()
num_edges = G.number_of_edges()
degrees = dict(G.degree())

print(f"Кількість вершин: {num_nodes}")
print(f"Кількість ребер: {num_edges}")
print("Ступінь вершин:", degrees)

# Завдання 2: Реалізація DFS і BFS

dfs_length = None
bfs_length = None
dijkstra_length = None

def dfs(graph, start):
    visited = set()
    stack = [start]
    path = []

    while stack:
        vertex = stack.pop()
        if vertex not in visited:
            visited.add(vertex)
            path.append(vertex)
            stack.extend(set(graph.neighbors(vertex)) - visited)
    
    return path

def bfs(graph, start):
    visited = set()
    queue = [start]
    path = []

    while queue:
        vertex = queue.pop(0)
        if vertex not in visited:
            visited.add(vertex)
            path.append(vertex)
            queue.extend(set(graph.neighbors(vertex)) - visited)
    
    return path

# Тестування алгоритмів
start_node = 'A'
dfs_path = dfs(G, start_node)
bfs_path = bfs(G, start_node)

print("DFS шлях:", dfs_path)
print("BFS шлях:", bfs_path)

# Завдання 3: Алгоритм Дейкстри

def dijkstra(graph, start):
    distances = {node: float('infinity') for node in graph.nodes()}
    distances[start] = 0
    priority_queue = [(0, start)]
    previous_nodes = {node: None for node in graph.nodes()}

    while priority_queue:
        current_distance, current_node = heapq.heappop(priority_queue)

        if current_distance > distances[current_node]:
            continue

        for neighbor in graph.neighbors(current_node):
            weight = graph[current_node][neighbor]['weight']
            distance = current_distance + weight

            if distance < distances[neighbor]:
                distances[neighbor] = distance
                previous_nodes[neighbor] = current_node
                heapq.heappush(priority_queue, (distance, neighbor))

    return distances, previous_nodes

# Виклик алгоритму Дейкстри
start_node = 'A'
shortest_paths, previous_nodes = dijkstra(G, start_node)

print("Найкоротші шляхи від A:", shortest_paths)

def reconstruct_path(previous_nodes, start, end):
    path = []
    while end is not None:
        path.append(end)
        end = previous_nodes[end]
    path.reverse()
    return path

# Розрахунок шляху з A до E
dijkstra_path = reconstruct_path(previous_nodes, start_node, 'E')

def calculate_path_length(graph, path):
    length = 0
    for i in range(len(path) - 1):
        if graph.has_edge(path[i], path[i + 1]):
            length += graph[path[i]][path[i + 1]]['weight']
        else:
            print(f"Немає ребра між {path[i]} та {path[i + 1]}.")
            return None  # або можна повернути 0 або інше значення
    return length

# Розрахунок довжини шляхів
dfs_length = calculate_path_length(G, dfs_path)
bfs_length = calculate_path_length(G, bfs_path)
dijkstra_length = calculate_path_length(G, dijkstra_path)

# Перевірка на None
if dfs_length is None:
    print("DFS шлях некоректний.")
else:
    print("DFS Path:", dfs_path)
    print("DFS Length:", dfs_length)

if bfs_length is None:
    print("BFS шлях некоректний.")
else:
    print("BFS Path:", bfs_path)
    print("BFS Length:", bfs_length)

if dijkstra_length is None:
    print("Dijkstra шлях некоректний.")
else:
    print("Dijkstra Path:", dijkstra_path)
    print("Dijkstra Length:", dijkstra_length)

# Порівняння довжин
print("Comparison of Path Lengths:")
if dfs_length is not None and bfs_length is not None and dijkstra_length is not None:
    if dfs_length < bfs_length and dfs_length < dijkstra_length:
        print("DFS found the shortest path.")
    elif bfs_length < dfs_length and bfs_length < dijkstra_length:
        print("BFS found the shortest path.")
    elif dijkstra_length < dfs_length and dijkstra_length < bfs_length:
        print("Dijkstra found the shortest path.")
    else:
        print("There is a tie in the shortest paths.")
else:
    print("Не вдалося порівняти довжини шляхів через некоректні значення.")

# Розрахунок довжини шляхів
dfs_length = calculate_path_length(G, dfs_path)
bfs_length = calculate_path_length(G, bfs_path)
dijkstra_length = calculate_path_length(G, dijkstra_path)

# Виведення результатів
print("DFS Path:", dfs_path)
print("DFS Length:", dfs_length)
print("BFS Path:", bfs_path)
print("BFS Length:", bfs_length)

print("Dijkstra Path:", dijkstra_path)
print("Dijkstra Length:", dijkstra_length)

