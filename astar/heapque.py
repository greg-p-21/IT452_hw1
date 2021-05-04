import heapq as hq 

class PriQue:
    def __init__(self):
        self.PQ = []

    def add(self, priority, item):
        hq.heappush(self.PQ, (priority, item))

    def get(self):
        return hq.heappop(self.PQ)[1]
    
if __name__ == "__main__":
    pq = PriQue()
    pq.add(5, 'greg')
    pq.add(3, 'c')
    pq.add(10, 'd')

    print(pq.get())
    print(pq.get())