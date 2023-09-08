import cv2 as cv
import numpy as np
import argparse
import math
import time
from collections import deque
from datetime import datetime

# Importing helper functions from stereo_utils module
from helper.stereo_utils import DLT, get_projection_matrix


# Define camera_to_world and z_needed globally
camera_to_world = 1
z_needed = 1

show_image =1
ligt_sensitivity= 60 #enter from 0-255

# if result video need to save to disk change write_video=1 else write_video=0
write_video = 0

def resize(frame, dst_width):
    width = frame.shape[1]
    height = frame.shape[0]
    scale = dst_width * 1.0 / width
    return cv.resize(frame, (int(scale * width), int(scale * height)))

class BallDetector:
    def __init__(self, device=0, width=None, height=None, fps=False, channel=-1):
        self.device = device
        self.width = width
        self.height = height
        self.fps = fps
        self.channel = channel
        self.camera_to_world = 1
        self.z_needed = 1
        self.arducam=0  #enable stereo camera or test through video 
        prev_x, prev_y = 0, 0
        self.prev_time = 0
        self.speed=0
        self.pts = deque(maxlen=10)
        #kfObj = KalmanFilter()
        #predictedCoords = np.zeros((2, 1), np.float32)
        self.count = 0
        self.i_time = 0
        self.l_time = 0
        self.prev_frame_time = time.time()
        # if result video need to save to disk change write_video=1 else write_video=0
        self.write_video = write_video

        if self.arducam==1:
            # Done on 80 cm ball video shared
            self.cap = cv.VideoCapture(0, cv.CAP_V4L2)
        else:
            self.cap = cv.VideoCapture("input_stereo_video.mp4")

        if self.arducam==1:
            from utils import ArducamUtils
            self.arducam_utils = ArducamUtils(self.device)

            # turn off RGB conversion
            if self.arducam_utils.convert2rgb == 0:
                self.cap.set(cv.CAP_PROP_CONVERT_RGB, self.arducam_utils.convert2rgb)
            # set width
            if self.width != None:
                self.cap.set(cv.CAP_PROP_FRAME_WIDTH, self.width)
            # set height
            if self.height != None:
                self.cap.set(cv.CAP_PROP_FRAME_HEIGHT, self.height)

            if self.channel in range(0, 4):
                self.arducam_utils.write_dev(ArducamUtils.CHANNEL_SWITCH_REG, self.channel)
        for i in range(3):
            ret, frame = self.cap.read()
            if not ret:
                print("no video")
                break

        if self.write_video:
            # initializing video writer
            width = int(840)
            height = int(280)
            fps = int(self.cap.get(cv.CAP_PROP_FPS))
            self.left_v = cv.VideoWriter('ball_output.avi', cv.VideoWriter_fourcc(*'XVID'), 30, (width, height))

    def __del__(self):
        # Release the video capture object when the instance is deleted
        self.cap.release()
        # Release video writer if saving video
        if self.write_video:
            self.left_v.release()

        cv.destroyAllWindows()

    def get_xyz(self, r_pt, l_pt):
        P0 = get_projection_matrix(0)
        P1 = get_projection_matrix(1)

        if l_pt[0] == 0 or r_pt[0] == 0:
            point_3d = [0, 0, 0]
        else:
            point_3d = DLT(P0, P1, r_pt, l_pt)  # calculate 3d position of keypoint

        return point_3d

    def circle_detect(self, frame):
        frame = frame.astype(np.uint8)
        frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

        th, thresh = cv.threshold(frame, ligt_sensitivity, 255, cv.THRESH_BINARY_INV)

        def draw_rect(frame):
            frame = cv.GaussianBlur(frame, (5, 5), 0)
            x, y, r = 0, 0, 0
            circles = cv.HoughCircles(frame, cv.HOUGH_GRADIENT, dp=1, minDist=20,
                                      param1=5, param2=5, minRadius=1, maxRadius=30)

            return circles

        detected_circles = draw_rect(thresh)

        return detected_circles, thresh

    def _map(self, x, in_min, in_max, out_min, out_max):
        a = int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)
        return a

    def process_frame(self):
        # Reading video source
        ret, frame = self.cap.read()
        if not ret:
            print("No video")
            #break
        if ret:
            if self.arducam == 1:
                if self.arducam_utils.convert2rgb == 0:
                    w = self.cap.get(cv.CAP_PROP_FRAME_WIDTH)
                    h = self.cap.get(cv.CAP_PROP_FRAME_HEIGHT)
                    frame = frame.reshape(int(h), int(w))

                frame = self.arducam_utils.convert(frame)
                frame = resize(frame, 960.0)

            # Slicing stereo video to left and right frames
            left_frame = frame[40:frame.shape[0] - 70, 30:(frame.shape[1] // 2) - 30]
            right_frame = frame[40:frame.shape[0] - 70, (frame.shape[1] // 2) + 30:frame.shape[1] - 30]

            if self.arducam==1:
                #left_frame=cv.cvtColor(left_frame,cv.COLOR_BGR2GRAY)
                left_frame=cv.cvtColor(left_frame, cv.COLOR_GRAY2BGR)
                right_frame=cv.cvtColor(right_frame, cv.COLOR_GRAY2BGR)
            return left_frame,right_frame
        return None,None

    def ball_detection(self,i_time): 
        
        if self.count != 1:
            if i_time is not None:
                #print(str(datetime.utcfromtimestamp(i_time)))
                self.i_time=i_time
            else:
                self.i_time=0

        #process video  
        left_frame,right_frame=self.process_frame()
            
        
        # Send left and right frames to ball detection function
        l_circles, thresh = self.circle_detect(left_frame)
        r_circles, _ = self.circle_detect(right_frame)
        thresh = cv.cvtColor(thresh, cv.COLOR_GRAY2BGR)

        r_org = (right_frame.shape[1]//2, 0)  # origin point in right frame
        l_org = (left_frame.shape[1]//2, 0)  # origin point in left frame

        def write_xy(lx, ly, lr, bZ):
            
            # # getting rectangle center
            lcenter = (lx, ly)
            # rcenter = (x+w//2, y+h//2)

            Lx, Ly, Lz = 0, 0, 0
            if lcenter != (0, 0):

                # mapping orgin point Y to the image's height to 0 to 100cm
                Ly = self._map(ly, l_org[1], left_frame.shape[0], 0, 86)

                # mapping orgin point X to the image's width to 0 to 100cm
                Lx = self._map(lx, l_org[0], left_frame.shape[1], 0, 120)

                if show_image:
                    # Display X and Y in frame
                    cv.putText(left_frame, "X: {} cm".format(str(round(Lx, 1))), (lx+lr+15, ly-30), cv.FONT_HERSHEY_SIMPLEX,
                            fontScale=1, color=[0, 0, 255], thickness=1, lineType=cv.LINE_AA)
                    cv.putText(left_frame, "Y: {} cm".format(str(round(Ly, 1))), (lx+lr+15, ly), cv.FONT_HERSHEY_SIMPLEX,
                            fontScale=1, color=[225, 0, 0], thickness=1, lineType=cv.LINE_AA)
                    cv.putText(left_frame, "Z: {} cm".format(str(round(bZ, 1))), (lx+lr+15, ly+30), cv.FONT_HERSHEY_SIMPLEX,
                            fontScale=1, color=[0, 225, 0], thickness=1, lineType=cv.LINE_AA)

            print("X,Y: ", Lx, Ly)
            # if (datetime.now().timestamp()-self.prev_time) > 300:
            #dist = math.dist((Lx, Ly), l_org)
            time_diff=((datetime.now().timestamp()-self.i_time))
            dist = Ly
            self.speed = float(dist/time_diff)
            # if self.speed > 300:
            #     self.speed = 0
            print(f"Dist: {dist} Spd: {self.speed} cm/s")
            # cv.putText(left_frame, "Spd: {} cm/s".format(str(round(self.speed,2))), (lx+lr+15, ly+60), cv.FONT_HERSHEY_SIMPLEX,
            #         fontScale=1, color=[0, 255, 255], thickness=1, lineType=cv.LINE_AA)
            prev_x, prev_y = Lx, Ly
            self.prev_time = datetime.now().timestamp()
            # update the points queue
            self.pts.appendleft(lcenter)

            if show_image:
                # loop over the set of tracked points
                for i in range(1, len(self.pts)):
                    # if either of the tracked points are None, ignore
                    # them

                    if self.pts[i - 1] is None or self.pts[i] is None:
                        continue

                    # otherwise, compute the thickness of the line and
                    # draw the connecting lines
                    thickness = int(np.sqrt(10 / float(i + 1)) * 2.5)
                    cv.line(left_frame, self.pts[i - 1], self.pts[i], (255, 0, 0), thickness)

        if l_circles is not None and r_circles is not None:

            l_circles = np.round(l_circles[0, :]).astype("int")
            r_circles = np.round(r_circles[0, :]).astype("int")
            # print(l_circles)
            if self.count == 0:
                l_time = 0
                self.count = 1

            if len(l_circles[0]) > 1:
                l_circles = [l_circles[0]]
            if len(r_circles[0]) > 1:
                r_circles = [r_circles[0]]
            # print(l_circles)

            # for (lx, ly, lr) in l_circles:
            # print("Radius:",lr)
            [lx, ly, lr] = l_circles[0]
            [rx, ry, rr] = r_circles[0]
            #predictedCoords = kfObj.Estimate(lx, ly)
            if show_image: cv.circle(left_frame, (lx, ly), lr, (0, 255, 0), 2)
            # cv.circle(left_frame, (int(predictedCoords[0]), int(
            #     predictedCoords[1])), lr, [0, 255, 255], 5, 8)
            #cv.line(left_frame, (int(predictedCoords[0]), int(predictedCoords[1])), (lx, ly), [100, 10, 255], 2, 8)

            # p1 = (int(predictedCoords[0]), int(predictedCoords[1]))
            # p2 = (lx, ly)

            # theta = np.arctan2(p1[1]-p2[1], p1[0]-p2[0])
            # endpt_x = int(p1[0] - 100*np.cos(theta))
            # endpt_y = int(p1[1] - 100*np.sin(theta))

            #cv.arrowedLine(left_frame, (p1[0], p1[1]), (endpt_x, endpt_y), [100, 10, 255], 2)

            #cv.putText(left_frame, "Predicted", (int(predictedCoords[0] + 50), int(predictedCoords[1] - 30)), cv.FONT_HERSHEY_SIMPLEX, 0.5, [50, 200, 250])
            # if (lr > 25 and lr <40):
            #     #pass
            #     cv.circle(left_frame, (lx, ly), lr, (0, 255, 0), 2)
            if z_needed:
                
                [bX, bY, bZ] = self.get_xyz((lx, ly), (rx, ry))
                # [OX, OY, OZ] = self.get_xyz(l_org, r_org)

                # ball reference from origin
                if camera_to_world == 0:
                    ball_left_xyz = self.get_xyz(
                        (lx-lr-50, ly), (rx-rr-50, ry))
                    ball_right_xyz = self.get_xyz(
                        (lx+lr+50, ly), (rx+rr+50, ry))
                    Lz = abs(bZ-((ball_left_xyz[2]+ball_right_xyz[2])//2))
                    # Lz=bZ-oZ

                # ball reference from camera
                else:
                    Lz = bZ
                    Lz= 90-Lz
                    if Lz <0: Lz=0
                #print([bX, bY, bZ], [OX, OY, OZ])
                write_xy(lx, ly, lr, Lz)
            else:

                write_xy(lx, ly, lr, 0)
            # else:
            #     cv.circle(left_frame, (lx, ly), lr, (0,0, 255), 2)

        else: #if circles not detected
            if self.count == 1:
                self.l_time = datetime.now().timestamp()
                #l_time = datetime.utcnow().strftime('%H:%M:%S.%f')[:-3]
            self.count = 0
            
            self.spd=[]
            self.avgspd=[]
            # if show_image:
            #     if self.l_time != 0 or self.i_time != 0 or self.l_time > self.i_time:
            #         cv.putText(left_frame, 'Leave time:'+str(datetime.utcfromtimestamp(self.l_time).strftime('%H:%M:%S.%f')[:-3]), (0, 65),
            #                 cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2, cv.LINE_AA)
                    
            #         if round(200/(self.l_time-self.i_time),2) < 5000 :
            #             cv.putText(left_frame, 'Avg spd:'+str(round(200/(self.l_time-self.i_time), 2))+'cm/s',
            #                     (0, 85), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv.LINE_AA)

            (lx, ly) = (0, 0)
            self.pts.clear()
            #predictedCoords = kfObj.Estimate(left_frame.shape[1]//2, 0)

        
        # Frame rate calculation and display
        new_frame_time = time.time()
        fps = (1 / (new_frame_time - self.prev_frame_time))
        self.prev_frame_time = new_frame_time
        #print(fps)
        
        if show_image:
            # Drawing reference axis on the image
            cv.line(left_frame, l_org, (l_org[0] + 50, l_org[1]), (0, 0, 255), 2)
            cv.line(left_frame, l_org, (l_org[0], l_org[1] + 50), (255, 0, 0), 2)

            cv.putText(left_frame, 'Current time:'+str(datetime.utcnow().strftime('%H:%M:%S.%f')[:-3]),
                    (0, 15), cv.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 1, cv.LINE_AA)
            cv.putText(left_frame, 'Impact time:'+str(datetime.utcfromtimestamp(self.i_time).strftime('%H:%M:%S.%f')[:-3]), (0, 30),
                    cv.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 1, cv.LINE_AA)
            cv.putText(left_frame, 'FPS: ' + str(round(fps, 2)), (0, 45), cv.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 1,
                    cv.LINE_AA)
            cv.putText(left_frame, 'Ball Speed: ' + str(round(self.speed, 2))+'cm/s', (0, 60), cv.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 1,
                    cv.LINE_AA)


            h_img = cv.hconcat([left_frame, thresh])
            cv.imshow("L", h_img)

        if self.write_video:
            self.left_v.write(h_img)

        if show_image: k = cv.waitKey(1)
        # if k == ord('q'):
        #     break
    

def main():
    parser = argparse.ArgumentParser(description='Ball Detection')
    parser.add_argument('-d', '--device', default=0, type=int, nargs='?',
                        help='/dev/videoX default is 0')
    parser.add_argument('--width', type=lambda x: int(x, 0),
                        help="set width of image")
    parser.add_argument('--height', type=lambda x: int(x, 0),
                        help="set height of image")
    parser.add_argument('--fps', action='store_true', help="display fps")
    parser.add_argument('--channel', type=int, default=-1, nargs='?',
                        help="When using Camarray's single channel, use this parameter to switch channels. \
                            (E.g. ov9781/ov9281 Quadrascopic Camera Bundle Kit)")
    args = parser.parse_args()

    ball_detector = BallDetector(device=args.device, width=args.width, height=args.height, fps=args.fps, channel=args.channel)
#     while 1:
#         ball_detector.ball_detection(0)

# if __name__ == "__main__":
#     main()