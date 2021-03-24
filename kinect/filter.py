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
    GRADIENT = 5

    def __init__(self, img, color, depth):
        self.img = img
        self.color = color
        self.depth = depth/Filter.GRADIENT
        self.mask = self.get_mask()

    def get_mask(self):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        mask = None
        if self.color == 'red':
            lower_red1 = np.array(Filter.HSV_COLOR_RANGES['red1'][1])
            upper_red1 = np.array(Filter.HSV_COLOR_RANGES['red1'][0])

            lower_red2 = np.array(Filter.HSV_COLOR_RANGES['red2'][1])
            upper_red2 = np.array(Filter.HSV_COLOR_RANGES['red2'][0])

            mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
            mask = mask1 + mask2
        else:
            lower = np.array(Filter.HSV_COLOR_RANGES[self.color][1])
            upper = np.array(Filter.HSV_COLOR_RANGES[self.color][0])
            mask = cv2.inRange(hsv, lower, upper)

        return mask

    def get_filtered(self):
        # self.mask = Filter.get_mask(img, color)
        img[self.mask != 0] = Filter.RGB_COLORS[self.color]
        return img

    def get_PID_value(self):
        mask_T = self.mask.T
        col_count = []

        for col in mask_T:
            col_count.append(np.count_nonzero(col))

        # find largest location
        max_column = np.argmax(col_count)
        # largest value
        max_amount = col_count[max_column]
        # number of columns
        num_columns = len(mask_T)

        return (num_columns/2 - max_column), max_amount

    def mean_distance(self):
        return np.nanmean(self.depth[self.mask != 0])


if __name__ == "__main__":
    imgs = ['left.png', 'right.png']
    for name in imgs:
        img = cv2.imread(name)
    	f_img, mask = Filter.get_mask(img, 'green')
        print(mask)
        # cv2.imshow("f_img", f_img)
        # cv2.waitKey(0)
        print(name)
        print(Filter.get_PID_value(mask, 'green'))
    
    print("done")