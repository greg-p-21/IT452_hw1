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
    def get_filtered(img, color):

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        mask = None
        if color == 'red':
            lower_red1 = np.array(Filter.HSV_COLOR_RANGES['red1'][1])
            upper_red1 = np.array(Filter.HSV_COLOR_RANGES['red1'][0])

            lower_red2 = np.array(Filter.HSV_COLOR_RANGES['red2'][1])
            upper_red2 = np.array(Filter.HSV_COLOR_RANGES['red2'][0])

            mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
            mask = mask1 + mask2
        else:
            lower = np.array(Filter.HSV_COLOR_RANGES[color][1])
            upper = np.array(Filter.HSV_COLOR_RANGES[color][0])
            mask = cv2.inRange(hsv, lower, upper)

        img[mask != 0] = Filter.RGB_COLORS[color]

        return img

    @staticmethod
    def get_PID_value(filtered_img, color):
        # num_ys = len(filtered_img[0])
        num_columns = len(filtered_img[0])
        # num_xs = len(filtered_img)
        rgb_color = np.asarray(Filter.RGB_COLORS[color])

        # color_columns = []
        # max_column = 0
        # max_amount = 0

        # # print(f_img)
        # for y in range(num_ys):
        #     amount = 0
        #     for x in range(num_xs):
        #         if (filtered_img[x,y] == rgb_color).all():
        #             amount = amount + 1

        #     color_columns.append(amount)
        #     if (amount > color_columns[max_column]):
        #         max_column = y
        #         max_amount = amount        

        # return (num_ys/2 - max_column), max_amount
        
        # count number of colored pixels in columns 
        # print(filtered_img == rgb_color)
        print(filtered_img.shape)
        color_columns = np.count_nonzero(filtered_img == rgb_color, axis=1 )
        print(color_columns)

        # find largest location
        max_column = np.argmax(color_columns)

        # largest value
        max_amount = color_columns[max_column]

        return (num_columns/2 - max_column), max_amount


if __name__ == "__main__":
    imgs = ['left.png', 'right.png']
    for name in imgs:
        img = cv2.imread(name)
    	f_img = Filter.get_filtered(img, 'green')
        cv2.imshow("f_img", f_img)
        cv2.waitKey(0)
        print(name)
        print(Filter.get_PID_value(f_img, 'green'))
    
    print("done")