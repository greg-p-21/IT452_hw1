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
        num_ys = len(filtered_img[0])
        num_xs = len(filtered_img)
        rgb_color = np.asarray(Filter.RGB_COLORS[color])

        color_columns = []
        max_column = 0
        max_amount = 0
        # print(f_img)
        for y in range(num_ys):
            amount = 0
            for x in range(num_xs):
                # print(type(filtered_img[x,y]))
                # print(filtered_img[x,y][0])
                if (filtered_img[x,y] == rgb_color).all():
                    amount = amount + 1

            color_columns.append(amount)
            if (amount > color_columns[max_column]):
                max_column = y
                max_amount = amount
        # print(max_column)

        return (max_column - num_ys/2), max_amount
        



if __name__ == '__main__':
    # colors = ['red', 'blue', 'green', 'purple', 'yellow']

    # for c in colors:
    #     name = c + '.png'
    #     f_img = Filter.get_filtered(name, c)
    #     print(c)
    #     print(Filter.get_PID_value(f_img, c))
    #     cv2.imshow(c, f_img)
    #     cv2.waitKey(0)

    color = 'red'
    pictures = ['redr.png', 'red.png', 'redr.png', 'nothing.png']
    for p in pictures:
        f_img = Filter.get_filtered(p, color)
        print(p)
        pid_val = Filter.get_PID_value(f_img, color)
        print(pid_val)
