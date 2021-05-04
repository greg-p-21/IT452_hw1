import heapq as hq 

class PQueue:
    def __init__(self):
        self.PQ = []
        self.entry_finder = {}
        self.REMOVED = '<removed-task>'
        # counter = itertools.count

    def add(self, priority, item):
        # update first 
        if item in self.entry_finder:
            self.remove(item)

        entry = [priority, item]
        self.entry_finder[item] = entry 

        hq.heappush(self.PQ, entry)

    def remove(item):
        entry = self.entry_finder.pop(item)
        entry[-1] = self.REMOVED

    def get(self):
        priority, item = hq.heappop(self.PQ)
        if item is not self.REMOVED:
            del self.entry_finder[item]
            return item

    def len(self):
        return len(self.PQ)
    
if __name__ == "__main__":
    pq = PriQue()
    pq.add(5, 'greg')
    pq.add(3, 'c')
    pq.add(10, 'd')

    print(pq.get())
    print(pq.get())