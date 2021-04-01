import numpy as np
import cv2

class augment:

    def __init__(self):
        self.color = 'white'
        while True:
            col = raw_input('Pick a balloon color: ')
            if col == 'Red' or col == 'red':
                col = 'r'
                break
            elif col == 'Green' or col == 'green':
                col = 'g'
                break
            elif col == 'Pink' or col == 'pink':
                col = 'p'
                break
            elif col == 'Yellow' or col == 'yellow':
                col = 'y'
                break
            elif col == 'Blue' or col == 'blue':
                col = 'b'
                break
            else:
                print('Not a valid option, pick again')
        self.color = col
        
    def identify(self,img):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        if self.color == 'r': #Red
            outhsv = cv2.inRange(hsv, np.array([0, 178, 101]), np.array([0, 200, 255]))
        elif self.color == 'g': #Green
            outhsv = cv2.inRange(hsv, np.array([35, 15, 20]), np.array([80, 255, 235]))

        elif self.color == 'p': #Pink
            outhsv = cv2.inRange(hsv, np.array([144, 203, 92]), np.array([162, 220, 255]))

        elif self.color == 'y': #Yellow
            outhsv = cv2.inRange(hsv, np.array([30, 229, 82]), np.array([30, 249, 255]))

        elif self.color == 'b': #Blue
            outhsv = cv2.inRange(hsv, np.array([106, 214, 82]), np.array([126, 234, 254]))

        #cv2.imshow('mask', outhsv)
        #cv2.waitKey(0)
        
        #creates/displays image where balloon requested is 'highlighted' 
        img[:,:,1] = np.bitwise_or(img[:,:,1], outhsv)
        cv2.imshow('augmented', img)
        cv2.waitKey(1)
        
        #switch row/col
        outhsv = self.switch(outhsv)
        return outhsv
    
    #switches the mask from being row based to being column based
    def switch(self, n):
        return n.T

    def median(self):
        return
