# Greg Polmatier
from math import ceil

def sortFunc(a, b, c):
    a = ceil(a)
    b = ceil(b)
    c = ceil(c)

    floats = [a,b,c]
    answer = []

    while len(floats) != 0:
        minimum = min(floats)
        floats.remove(minimum)
        answer.append(minimum)

    return map(int, answer)
    
one = input("Num 1: ")
two = input("Num 2: ")
three = input("Num 3: ")

print "The ceiling of those numbers sorted are:", sortFunc(one,two,three)

