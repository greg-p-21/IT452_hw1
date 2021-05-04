###implementation of PriorityQueue
class PriorityQueue(object):
    def __init__(self):
        self.PQ={}

    def empty(self):
        return len(self.PQ) == 0

    def add(self,number,item):
        self.PQ[item] = number

    def get(self):
        minKey = next(iter(self.PQ))
        min = self.PQ[minKey]
        for eachKey in self.PQ.keys():
            if(self.PQ[eachKey]<min):
                min = self.PQ[eachKey]
                minKey = eachKey
        self.PQ.pop(minKey)
        return  minKey
    def len(self):
        return len(self.PQ)

###########################################################
'''
test = PriorityQueue()
test.add(10,'A')
test.add(11,'B')
test.add(2,'C')
print(test.get())
'''
