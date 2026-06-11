import heapq

# S = Inicio
# G = Final
def preprocess_maze(maze):
    """Parsea la matriz para encontrar S y G, devolviendo una matriz numérica."""
    num_maze = []
    start = None
    goal = None
    
    for r, row in enumerate(maze):
        num_row = []
        for c, val in enumerate(row):
            if val == 'S':
                start = (r, c)
                num_row.append(0) # S es caminable
            elif val == 'G':
                goal = (r, c)
                num_row.append(0) # G es caminable
            else:
                num_row.append(int(val))
        num_maze.append(num_row)
        
    return num_maze, start, goal

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def a_star(maze, start, goal):
    filas, columnas = len(maze), len(maze[0])
    movimientos = [(-1, 0), (1, 0), (0, -1), (0, 1)] # Arriba, Abajo, Izquierda, Derecha
    
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}
    
    while open_set:
        current = heapq.heappop(open_set)[1]
        
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]
            
        for dx, dy in movimientos:
            vecino = (current[0] + dx, current[1] + dy)
            
            if 0 <= vecino[0] < filas and 0 <= vecino[1] < columnas and maze[vecino[0]][vecino[1]] == 0:
                tentative_g_score = g_score[current] + 1
                
                if vecino not in g_score or tentative_g_score < g_score[vecino]:
                    came_from[vecino] = current
                    g_score[vecino] = tentative_g_score
                    f_score[vecino] = tentative_g_score + heuristic(vecino, goal)
                    heapq.heappush(open_set, (f_score[vecino], vecino))
                    
    return [] # Sin solución
