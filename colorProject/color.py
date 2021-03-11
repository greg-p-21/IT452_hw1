import cv2
import numpy as np

class Filter:
    HSV_COLOR_RANGES = {
        'red1': [[180, 255, 255], [159, 50, 70]],
        'red2': [[9, 255, 255], [0, 50, 70]],
        'green': [[89, 255, 255], [36, 50, 70]],
        'blue': [[128, 255, 255], [90, 50, 70]],
        'yellow': [[35, 255, 255], [25, 50, 70]],
        'purple': [[158, 255, 255], [129, 50, 70]]
    }

    RGB_COLORS = {
        'red': [0,0,255],
        'green': [0,255,0],
        'blue': [255,0,0],
        'purple': [255,0,255],
        'yellow': [0,255,255]
    }

    @staticmethod
    def get_filtered(imgname, color):
        img = cv2.imread(imgname)

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        mask = None
        if color == 'red':
            mask1 = cv2.inRange(hsv, np.array(HSV_COLOR_RANGES['red1'][1]), np.array(HSV_COLOR_RANGES['red1'][0]))
            mask2 = cv2.inRange(hsv, np.array(HSV_COLOR_RANGES['red2'][1]), np.array(HSV_COLOR_RANGES['red2'][0]))

            mask = mask1+mask2
        else:
            mask = cv2.inRange(hsv, np.array(HSV_COLOR_RANGES[color][1]), np.array(HSV_COLOR_RANGES[color][0]))
            
        # The black region in the mask has the value of 0,
        # so when multiplied with original image removes all non-blue regions
        # result = cv2.bitwise_and(img, img, mask = mask)


        img[mask != 0] = RGB_COLORS[color]

        return img

if __name__ == '__main__':
    yellow = Filter.get_filtered('yellow.png', 'yellow')
    cv2.imshow("yellow", yellow)
    cv2.waitKey(0)

    red = Filter.get_filtered('red.png', 'red')
    cv2.imshow('red', red)
    cv2.waitKey(0)
