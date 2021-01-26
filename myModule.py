class MyClass:
    def __init__(self, l):
        self.l = l

    def odds(self):
        odd = []
        for i in range(len(self.l)):
            if i % 2 == 1:
                odd.append(self.l[i])

        return odd

    def oddsPlusC(self, c):
        odd = self.odds()
        for i in range(len(odd)):
            odd[i] += c
        return odd

    
