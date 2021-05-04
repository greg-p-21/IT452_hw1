import sys
import copy


class PriQue(object):

    # A class for the linked list
    class Node(object):
        def __init__(self, data):
            self.data = data
            self.next = None
            self.prev = None


    def __init__(self):
        self.head = None
        self.min = None
        self.dic = {}
        self.size = 0

    # A method that inserts data at the front of the list and keeps a pointer
    # to the minimum val node
    def put(self, data):
        nNode = self.Node(data)

        # Use dictionary to update values if necessary
        # Do not add redundant states
        if self.dic.get(data[0]) is None:
            self.dic[data[0]] = data[1]
            self.size = self.size + 1
        elif self.dic.get(data[0]) >= data[1]:
            self.dic[data[0]] = data[1]
            self.update(data)
            return
        else:
            return

        # Update min if necessary
        if self.min is not None:
            if data[1] < self.min.data[1]:
                self.min = nNode
        else:
            self.min = nNode

        # Insert to front
        if self.head is None:
            nNode.next = None
            self.head = nNode
        else:
            nNode.next = self.head
            self.head.prev = nNode
            self.head = nNode


    # A method that removes the minimum fval state from the list and returns
    # its data, then finds the new minimum fval in the list and points min to it
    def removeMin(self):
        if self.min is None:
            return "Priority Queue is empty"

        self.size = self.size - 1

        # Remove the min and set new min
        minNode = self.remove(self.min)
        if self.head is not None:
            minVal = self.findMinVal()
            self.setMin(minVal)
        return minNode

    # A method that removes a specified node from the list
    def remove(self, n):

        # Reset min pointer
        self.min = None

        # If there is only one node in list
        if n.prev is None and n.next is None:
            self.head = None
            return n.data

        # If it is removing the first node
        if n.prev is None:
            self.head = n.next
            n.next.prev = None
            return n.data

        # If it is removing the last node
        if n.next is None:
            n.prev.next = None
        else:
            n.prev.next = n.next
            n.next.prev = n.prev

        return n.data

    # A method that finds the minimum val in the list
    def findMinVal(self):
        min = self.head.data[1]
        n = self.head.next

        while n is not None:
            if n.data[1] < min:
                min = n.data[1]
            n = n.next

        return min

    # A method that sets the min pointer to the node (or one of the nodes) with
    # the minimum fval
    def setMin(self, min):
        n = self.head

        while n is not None:
            if n.data[1] == min:
                self.min = n
                break
            n = n.next

    def isNotEmpty(self):
        return self.head is not None


    def update(self, data):
        n = self.head

        while n is not None:
            if n.data[0] == data[0]:
                n.data = copy.copy(data)
                break
            n = n.next
