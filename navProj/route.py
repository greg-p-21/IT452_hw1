from PriQue import *
import copy
from adjMatrix import AdjMatrix

# Primary A* function
def dijkstras(adj_matx, start, finish):
    # adj_matx = AdjMatrix(dotfile, start[1], finish[1])

    # Add start to Priority Queue
    q = PriQue()
    dist = {}
    prev = {}

    # Set to track closed nodes
    closed = set()
    visited = set()

    # Initialize start
    s = start
    visited.add(s[0])
    q.put(start)
    dist[s[0]] = 0
    prev[s[0]] = None

    # Perform Dijkstra's
    while q.isNotEmpty():
        s = q.removeMin()

        # Generate possible moves from s
        neighbors = adj_matx.getNeighbors(s[0])

        # Perform possible moves on s and record as children
        for n in neighbors:
            if n[0] in closed:
                continue

            weight = dist[s[0]] + n[1]

            if not n[0] in visited:
                visited.add(n[0])
                prev[n[0]] = s[0]
                dist[n[0]] = weight
            elif weight < dist[n[0]]:
                dist[n[0]] = weight
                prev[n[0]] = s[0]

            q.put(n)

        # Add s to closed
        closed.add(s[0])

    # Get shortest path from finish
    route = []
    trace = finish[0]
    while trace:
        route.append(trace)
        trace = prev[trace]

    route.reverse()

    return route, adj_matx.getPoints(route)


if __name__ == "__main__":
    start = ("start", (0,0))
    end = ("finish", (-21,4))

    adj = AdjMatrix("Hopper.DOT", start[1], end[1])

    route, points = dijkstras(adj, start, end)
    print(route, points)