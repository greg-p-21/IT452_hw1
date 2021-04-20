from PriQue import *
import copy

def getNeighbors(adj_matx, s):
    pass

# Primary A* function
def dijkstras(adj_matx, start, finish):

    # Add start to Priority Queue
    q = PriQue()
    dist = {}
    route = {}
    q.put(start)

    # Set to track closed nodes
    closed = set()
    visited = set()

    s = start
    visited.add(s[0])

    while True:
        s = q.removeMin()

        # Generate possible moves from s
        neighbors = getNeighbors(adj_matx, s)

        # Perform possible moves on s and record as children
        for n in neighbors:
            visited.add(n[0])
            if not n[0] in closed:
                q.put(n)
                weight = dist[s[0]] + n[1]
                if n[0] in dist:
                    if weight < dist[n[0]]:
                        dist[n[0]] = weight
                else:
                    dist[n[0]] = weight

                if n[0] in route:
                    route[n[0]].append(s[0])
                else:
                    route[n[0]] = [s[0]]

        # Add s to closed
        closed.add(s[0])

        if s[0] == finish[0]:
            return route[s[0]]
