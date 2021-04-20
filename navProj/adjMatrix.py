# MIDN 1/C Polmatier
import math
from pprint import pprint


class AdjMatrix:

    def __init__(self, dotfile, start, finish):
        self.points = []
        self.dotfile = dotfile
        self.start = start 
        self.finish = finish
        self.matrix = self._toMatrix()
        

    def _toMatrix(self):
        matrix = {}

        f = open(self.dotfile, "r")      
        flines = f.readlines()
        for line in flines:
            if 'graph' in line:
                continue

            elif 'label' in line:
                values = line.split()
                label = values[0]
                loc = values[3].split('"')[1][1:-1]
                point = tuple(float(num) for num in loc.split(','))

                self.points.append(Point(label, point))

            elif '--' in line:
                values = line.split()
                label1 = values[0]
                label2 = values[2][0]
                distance = self.getDistance(label1, label2)

                if label1 not in matrix:
                    matrix[label1] = {}
                
                if label2 not in matrix:
                    matrix[label2] = {}

                matrix[label1][label2] = distance
                matrix[label2][label1] = distance

        strt = Point('start', self.start)
        end = Point('finish', self.finish)

        matrix['start'] = {}
        matrix['finish'] = {}

        matrix = self._addEdge(matrix, strt)
        matrix = self._addEdge(matrix, end)

        self.points.append(strt)
        self.points.append(end)

        pprint(matrix)
        return matrix

    def _addEdge(self, matrix, newPoint):
        min_dist = 1000
        min_point = None
        for p in self.points:
            dist = Point.calcDistance(p, newPoint)
            if min_dist > dist:
                min_dist = dist
                min_point = p 

        matrix[newPoint.getLabel][min_point.getLabel] = min_dist
        matrix[min_point.getLabel][newPoint.getLabel] = min_dist
        
        return matrix


    def getDistance(self, a, b):
        pa = None
        pb = None
        for p in self.points:
            if p.getLabel == a:
                pa = p
            if p.getLabel == b:
                pb = p
        
        return Point.calcDistance(pa, pb)

    def getNeighbors(self, label):
        neighbors = []
        for n in self.matrix[label].items():
            neighbors.append(n)

        return neighbors


class Point:

    def __init__(self, label, location):
        self.label = label
        self.location = location

    @property
    def getX(self):
        return self.location[0]

    @property
    def getY(self):
        return self.location[1]

    @property
    def getLabel(self):
        return self.label

    @staticmethod
    def calcDistance(p1, p2):
        return math.sqrt( pow(p1.getX - p2.getX, 2) + pow(p1.getY - p2.getY, 2) )

if __name__ == "__main__":
    adj = AdjMatrix("example.DOT", (0,0), (6,4))
    pprint(adj.getNeighbors('C'))