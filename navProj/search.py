<<<<<<< HEAD

def dijk_search(graph, source):
=======
import heapq

SOURCE = "start"
TARGET = "finish"

def dijk_search(graph):
    dist = {}
    prev = {}

    dist[SOURCE] = 0

    for vertex in graph.getVertices():
        if vertex is not SOURCE:
            dist[vertex] = float('infinity')
            prev[vertex] = None
        
    pq = [(0, SOURCE)]

    while len(pq) > 0:
        curr_dist, curr_vertex = heapq.heappop(pq)

        if curr_vertex == TARGET:
            path = get_path(prev)
            points = adj.getPoints(path)
            return path, points

        if curr_dist > dist[curr_vertex]:
            continue

        for n, weight in graph.getNeighbors(curr_vertex):
            distance = curr_dist + weight
            if distance < dist[n]:
                dist[n] = distance
                prev[n] = curr_vertex
                heapq.heappush(pq, (distance, n))
    
    path = get_path(prev)
    points = adj.getPoints(path)
    return path, points

def get_path(prev_list):
    S = []
    u = TARGET
    while u is not None and u is not 'start':
        S.insert(0, u)
        u = prev_list[u]
    
    S.insert(0, 'start')

    return S





from adjMatrix import AdjMatrix

if __name__ == "__main__":
    adj = AdjMatrix('example.DOT', (0,0), (6,4))

    from pprint import pprint

    pprint(dijk_search(adj))

        

>>>>>>> 458b98d7deb715bd76a745077a6b162fab48341b
