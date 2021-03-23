import math

class AnglePID:
    def __init__(self, k):
        self.k = k

     def get_value(self, currAngle, targetAngle, k):
        angleVel = angleDiff - curr_yaw

        scaledAngle = AnglePID.scale_angle(angleVel)

        return scaledAngle*k

    @staticmethod
    def scale_angle(angle):
        if angle < -math.pi/4 or angle > math.pi/4:
            if target_y < 0 and curr_y < target_y:
                return -2*math.pi + angle
            elif target_y >= 0 and curr_y > target_y:
                return 2*math.pi + angle 

        return angle