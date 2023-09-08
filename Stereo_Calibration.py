import cv2 as cv
import glob
import numpy as np
import sys
from scipy import linalg
import yaml
import os
from helper.utils import read_camera_parameters
from utils import ArducamUtils
# This will contain the calibration settings from the calibration_settings.yaml file
calibration_settings = {}

arducam_utils = ArducamUtils('0')


# Given Projection matrices P1 and P2, and pixel coordinates point1 and point2, return triangulated 3D point.


def DLT(P1, P2, point1, point2):

    A = [point1[1]*P1[2, :] - P1[1, :],
         P1[0, :] - point1[0]*P1[2, :],
         point2[1]*P2[2, :] - P2[1, :],
         P2[0, :] - point2[0]*P2[2, :]
         ]
    A = np.array(A).reshape((4, 4))

    B = A.transpose() @ A
    U, s, Vh = linalg.svd(B, full_matrices=False)

    #print('Triangulated point: ')
    # print(Vh[3,0:3]/Vh[3,3])
    return Vh[3, 0:3]/Vh[3, 3]


# Open and load the calibration_settings.yaml file
def parse_calibration_settings_file(filename):

    global calibration_settings

    if not os.path.exists(filename):
        print('File does not exist:', filename)
        quit()

    print('Using for calibration settings: ', filename)

    with open(filename) as f:
        calibration_settings = yaml.safe_load(f)

    # rudimentray check to make sure correct file was loaded
    if 'camera0' not in calibration_settings.keys():
        print('camera0 key was not found in the settings file. Check if correct calibration_settings.yaml file was passed')
        quit()

def resize(frame, dst_width):
    width = frame.shape[1]
    height = frame.shape[0]
    scale = dst_width * 1.0 / width
    return cv.resize(frame, (int(scale * width), int(scale * height)))


# Open camera stream and save frames
def save_frames_single_camera(single_source,camera_name):

    # create frames directory
    if not os.path.exists('frames'):
        os.mkdir('frames')

    # get settings
    camera_device_id = calibration_settings[camera_name]
    width = calibration_settings['frame_width']
    height = calibration_settings['frame_height']
    number_to_save = calibration_settings['mono_calibration_frames']
    view_resize = calibration_settings['view_resize']

    # open video stream and change resolution.
    # Note: if unsupported resolution is used, this does NOT raise an error.
    cap = cv.VideoCapture(camera_device_id)
    #cap.set(3, width)
    #cap.set(4, height)
    
    # turn off RGB conversion
    if arducam_utils.convert2rgb == 0:
        cap.set(cv.CAP_PROP_CONVERT_RGB, arducam_utils.convert2rgb)
    # # set width
    # if args.width != None:
    #     cap.set(cv.CAP_PROP_FRAME_WIDTH, args.width)
    # # set height
    # if args.height != None:
    #     cap.set(cv.CAP_PROP_FRAME_HEIGHT, args.height)

    # if args.channel in range(0, 4):
    #     arducam_utils.write_dev(ArducamUtils.CHANNEL_SWITCH_REG, args.channel)

    start = False
    saved_count = 0

    while True:

        ret, frame = cap.read()
        if ret == False:
            # if no video data is received, can't calibrate the camera, so exit.
            print("No video data received from camera. Exiting...")
            quit()
        
        if arducam_utils.convert2rgb == 0:
            w = cap.get(cv.CAP_PROP_FRAME_WIDTH)
            h = cap.get(cv.CAP_PROP_FRAME_HEIGHT)
            frame = frame.reshape(int(h), int(w))

        frame = arducam_utils.convert(frame)
        frame = resize(frame, 1280.0)

        if single_source:
            if camera_name == 'camera0':
                frame = frame[0:frame.shape[0], 0:(frame.shape[1]//2)]
            if camera_name == 'camera1':
                frame = frame[0:frame.shape[0], (frame.shape[1]//2):frame.shape[1]]

        frame_small = cv.resize(
            frame, None, fx=1/view_resize, fy=1/view_resize)

        if not start:
            if not str(camera_device_id).isnumeric():
                cv.putText(frame_small, "Press any key to update frame and press SPACEBAR to start collection frames",
                        (50, 50), cv.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 1)
            else:
                cv.putText(frame_small, "press SPACEBAR to start collection frames",
                        (50, 50), cv.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 1)

        cv.putText(frame_small, "Num frames: " + str(saved_count),
                    (50, 100), cv.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 1)

        cv.imshow('frame_small', frame_small)
        if not str(camera_device_id).isnumeric(): k = cv.waitKey(0)
        else: k = cv.waitKey(1)

        if k == 27:
            # if ESC is pressed at any time, the program will exit.
            quit()

        if k == 32:
            # Press spacebar to start data collection
            savename = os.path.join(
                'frames', camera_name + '_' + str(saved_count) + '.png')
            cv.imwrite(savename, frame)
            saved_count += 1
            

        # break out of the loop when enough number of frames have been saved
        if saved_count == number_to_save:
            break

    cv.destroyAllWindows()


# Calibrate single camera to obtain camera intrinsic parameters from saved frames.
def calibrate_camera_for_intrinsic_parameters(images_prefix):

    # NOTE: images_prefix contains camera name: "frames/camera0*".
    images_names = glob.glob(images_prefix)

    # read all frames
    images = [cv.imread(imname, 1) for imname in images_names]

    # criteria used by checkerboard pattern detector.
    # Change this if the code can't find the checkerboard.
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 100, 0.001)

    rows = calibration_settings['checkerboard_rows']
    columns = calibration_settings['checkerboard_columns']
    # this will change to user defined length scale
    world_scaling = calibration_settings['checkerboard_box_size_scale']

    # coordinates of squares in the checkerboard world space
    objp = np.zeros((rows*columns, 3), np.float32)
    objp[:, :2] = np.mgrid[0:rows, 0:columns].T.reshape(-1, 2)
    objp = world_scaling * objp

    # frame dimensions. Frames should be the same size.
    width = images[0].shape[1]
    height = images[0].shape[0]

    # Pixel coordinates of checkerboards
    imgpoints = []  # 2d points in image plane.

    # coordinates of the checkerboard in checkerboard world space.
    objpoints = []  # 3d point in real world space

    for i, frame in enumerate(images):
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

        # find the checkerboard
        ret, corners = cv.findChessboardCorners(gray, (rows, columns), None)

        if ret == True:

            # Convolution size used to improve corner detection. Don't make this too large.
            conv_size = (11, 11)

            # opencv can attempt to improve the checkerboard coordinates
            corners = cv.cornerSubPix(
                gray, corners, conv_size, (-1, -1), criteria)
            cv.drawChessboardCorners(frame, (rows, columns), corners, ret)
            cv.putText(frame, 'If detected points are poor, press "s" to skip this sample',
                       (25, 25), cv.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 1)

            cv.imshow('img', frame)
            k = cv.waitKey(0)

            if k & 0xFF == ord('s'):
                print('skipping')
                continue

            objpoints.append(objp)
            imgpoints.append(corners)

    cv.destroyAllWindows()
    ret, cmtx, dist, rvecs, tvecs = cv.calibrateCamera(
        objpoints, imgpoints, (width, height), None, None)
    print('rmse:', ret)
    print('camera matrix:\n', cmtx)
    print('distortion coeffs:', dist)

    return cmtx, dist

# save camera intrinsic parameters to file


def save_camera_intrinsics(camera_matrix, distortion_coefs, camera_name):

    # create folder if it does not exist
    if not os.path.exists('camera_parameters'):
        os.mkdir('camera_parameters')

    out_filename = os.path.join(
        'camera_parameters', camera_name + '_intrinsics.dat')
    outf = open(out_filename, 'w')

    outf.write('intrinsic:\n')
    for l in camera_matrix:
        for en in l:
            outf.write(str(en) + ' ')
        outf.write('\n')

    outf.write('distortion:\n')
    for en in distortion_coefs[0]:
        outf.write(str(en) + ' ')
    outf.write('\n')


# open both cameras and take calibration frames
def save_frames_two_cams(single_source, camera0_name, camera1_name):

    # create frames directory
    if not os.path.exists('frames_pair'):
        os.mkdir('frames_pair')

    # settings for taking data
    view_resize = calibration_settings['view_resize']
    number_to_save = calibration_settings['stereo_calibration_frames']

    # open the video streams
    if not single_source:
        cap0 = cv.VideoCapture(calibration_settings[camera0_name])
        cap1 = cv.VideoCapture(calibration_settings[camera1_name])
    else:
        cap0 = cv.VideoCapture(calibration_settings[camera0_name])

    # set camera resolutions
    width = calibration_settings['frame_width']
    height = calibration_settings['frame_height']
    # cap0.set(3, width)
    # cap0.set(4, height)
    # cap1.set(3, width)
    # cap1.set(4, height)
    if arducam_utils.convert2rgb == 0:
        cap0.set(cv.CAP_PROP_CONVERT_RGB, arducam_utils.convert2rgb)

    start = False
    saved_count = 0
    while True:
        
        if not single_source:
            ret0, frame0 = cap0.read()
            ret1, frame1 = cap1.read()

            if not ret0 or not ret1:
                print('Cameras not returning video data. Exiting...')
                quit()
        else: 
            ret, frame = cap0.read()
            if ret == False:
                # if no video data is received, can't calibrate the camera, so exit.
                print("No video data received from camera. Exiting...")
                quit()

            if arducam_utils.convert2rgb == 0:
                w = cap0.get(cv.CAP_PROP_FRAME_WIDTH)
                h = cap0.get(cv.CAP_PROP_FRAME_HEIGHT)
                frame = frame.reshape(int(h), int(w))

            frame = arducam_utils.convert(frame)
            frame = resize(frame, 1280.0)

            frame0 = frame[0:frame.shape[0], 0:(frame.shape[1]//2)]
            frame1 = frame[0:frame.shape[0], (frame.shape[1]//2):frame.shape[1]]
           
        frame0_small = cv.resize(
            frame0, None, fx=1./view_resize, fy=1./view_resize)
        frame1_small = cv.resize(
            frame1, None, fx=1./view_resize, fy=1./view_resize)

        if not start:
            cv.putText(frame0_small, "Make sure both cameras can see the calibration pattern well",
                       (50, 50), cv.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 1)
            if not str(calibration_settings[camera0_name]).isnumeric():
                cv.putText(frame0_small, "Press any key to update frame and Press any key to SPACEBAR to start collection frames",
                    (50, 100), cv.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 1)
                    
            else:
                cv.putText(frame0_small, "Press any key to SPACEBAR to start collection frames",
                        (50, 100), cv.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 1)

        # if start:
        cv.putText(frame0_small, "Num frames: " + str(saved_count),
                    (50, 100), cv.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 1)
        cv.putText(frame1_small, "Num frames: " + str(saved_count),
                    (50, 100), cv.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 1)

        cv.imshow('frame0_small', frame0_small)
        cv.imshow('frame1_small', frame1_small)
        #if not str(calibration_settings[camera0_name]).isnumeric(): 
        k = cv.waitKey(0)
        #else: k = cv.waitKey(1)

        if k == 27:
            # if ESC is pressed at any time, the program will exit.
            quit()

        if k == 32:
            # Press spacebar to start data collection
            savename = os.path.join(
                'frames_pair', camera0_name + '_' + str(saved_count) + '.png')
            cv.imwrite(savename, frame0)

            savename = os.path.join(
                'frames_pair', camera1_name + '_' + str(saved_count) + '.png')
            cv.imwrite(savename, frame1)

            saved_count += 1

        # break out of the loop when enough number of frames have been saved
        if saved_count == number_to_save:
            break

    cv.destroyAllWindows()


# open paired calibration frames and stereo calibrate for cam0 to cam1 coorindate transformations
def stereo_calibrate(mtx0, dist0, mtx1, dist1, frames_prefix_c0, frames_prefix_c1):
    # read the synched frames
    c0_images_names = sorted(glob.glob(frames_prefix_c0))
    c1_images_names = sorted(glob.glob(frames_prefix_c1))

    # open images
    c0_images = [cv.imread(imname, 1) for imname in c0_images_names]
    c1_images = [cv.imread(imname, 1) for imname in c1_images_names]

    # change this if stereo calibration not good.
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 100, 0.001)

    # calibration pattern settings
    rows = calibration_settings['checkerboard_rows']
    columns = calibration_settings['checkerboard_columns']
    world_scaling = calibration_settings['checkerboard_box_size_scale']

    # coordinates of squares in the checkerboard world space
    objp = np.zeros((rows*columns, 3), np.float32)
    objp[:, :2] = np.mgrid[0:rows, 0:columns].T.reshape(-1, 2)
    objp = world_scaling * objp

    # frame dimensions. Frames should be the same size.
    width = c0_images[0].shape[1]
    height = c0_images[0].shape[0]

    # Pixel coordinates of checkerboards
    imgpoints_left = []  # 2d points in image plane.
    imgpoints_right = []

    # coordinates of the checkerboard in checkerboard world space.
    objpoints = []  # 3d point in real world space

    for frame0, frame1 in zip(c0_images, c1_images):
        gray1 = cv.cvtColor(frame0, cv.COLOR_BGR2GRAY)
        gray2 = cv.cvtColor(frame1, cv.COLOR_BGR2GRAY)
        c_ret1, corners1 = cv.findChessboardCorners(
            gray1, (rows, columns), None)
        c_ret2, corners2 = cv.findChessboardCorners(
            gray2, (rows, columns), None)

        if c_ret1 == True and c_ret2 == True:

            corners1 = cv.cornerSubPix(
                gray1, corners1, (11, 11), (-1, -1), criteria)
            corners2 = cv.cornerSubPix(
                gray2, corners2, (11, 11), (-1, -1), criteria)

            p0_c1 = corners1[0, 0].astype(np.int32)
            p0_c2 = corners2[0, 0].astype(np.int32)

            cv.putText(frame0, 'O', (p0_c1[0], p0_c1[1]),
                       cv.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 1)
            cv.drawChessboardCorners(frame0, (rows, columns), corners1, c_ret1)
            cv.imshow('img', frame0)

            cv.putText(frame1, 'O', (p0_c2[0], p0_c2[1]),
                       cv.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 1)
            cv.drawChessboardCorners(frame1, (rows, columns), corners2, c_ret2)
            cv.imshow('img2', frame1)
            k = cv.waitKey(0)

            if k & 0xFF == ord('s'):
                print('skipping')
                continue

            objpoints.append(objp)
            imgpoints_left.append(corners1)
            imgpoints_right.append(corners2)

    stereocalibration_flags = cv.CALIB_FIX_INTRINSIC
    ret, CM1, dist0, CM2, dist1, R, T, E, F = cv.stereoCalibrate(objpoints, imgpoints_left, imgpoints_right, mtx0, dist0,
                                                                 mtx1, dist1, (width, height), criteria=criteria, flags=stereocalibration_flags)

    print('rmse: ', ret)
    cv.destroyAllWindows()
    return R, T

# Converts Rotation matrix R and Translation vector T into a homogeneous representation matrix


def _make_homogeneous_rep_matrix(R, t):
    P = np.zeros((4, 4))
    P[:3, :3] = R
    P[:3, 3] = t.reshape(3)
    P[3, 3] = 1

    return P
# Turn camera calibration data into projection matrix


def get_projection_matrix(cmtx, R, T):
    P = cmtx @ _make_homogeneous_rep_matrix(R, T)[:3, :]
    return P

# After calibrating, we can see shifted coordinate axes in the video feeds directly


def check_calibration(single_source, camera0_name, camera0_data, camera1_name, camera1_data, _zshift=50.):

    cmtx0 = np.array(camera0_data[0])
    dist0 = np.array(camera0_data[1])
    R0 = np.array(camera0_data[2])
    T0 = np.array(camera0_data[3])
    cmtx1 = np.array(camera1_data[0])
    dist1 = np.array(camera1_data[1])
    R1 = np.array(camera1_data[2])
    T1 = np.array(camera1_data[3])

    P0 = get_projection_matrix(cmtx0, R0, T0)
    P1 = get_projection_matrix(cmtx1, R1, T1)

    # define coordinate axes in 3D space. These are just the usual coorindate vectors
    coordinate_points = np.array([[0., 0., 0.],
                                  [1., 0., 0.],
                                  [0., 1., 0.],
                                  [0., 0., 1.]])
    z_shift = np.array([0., -34., _zshift]).reshape((1, 3))
    # increase the size of the coorindate axes and shift in the z direction
    draw_axes_points = 5 * coordinate_points + z_shift
    print(draw_axes_points)
    # project 3D points to each camera view manually. This can also be done using cv.projectPoints()
    # Note that this uses homogenous coordinate formulation
    pixel_points_camera0 = []
    pixel_points_camera1 = []
    for _p in draw_axes_points:
        X = np.array([_p[0], _p[1], _p[2], 1.])

        # project to camera0
        uv = P0 @ X
        uv = np.array([uv[0], uv[1]])/uv[2]
        pixel_points_camera0.append(uv)

        # project to camera1
        uv = P1 @ X
        uv = np.array([uv[0], uv[1]])/uv[2]
        pixel_points_camera1.append(uv)

    # these contain the pixel coorindates in each camera view as: (pxl_x, pxl_y)
    pixel_points_camera0 = np.array(pixel_points_camera0)
    pixel_points_camera1 = np.array(pixel_points_camera1)

    # open the video streams
    if not single_source:
        cap0 = cv.VideoCapture(calibration_settings[camera0_name])
        cap1 = cv.VideoCapture(calibration_settings[camera1_name])
    else:
        cap0 = cv.VideoCapture(calibration_settings[camera0_name])

    # set camera resolutions
    width = calibration_settings['frame_width']
    height = calibration_settings['frame_height']
    # cap0.set(3, width)
    # cap0.set(4, height)
    # cap1.set(3, width)
    # cap1.set(4, height)
    if arducam_utils.convert2rgb == 0:
        cap0.set(cv.CAP_PROP_CONVERT_RGB, arducam_utils.convert2rgb)
    while True:

        if not single_source:
            ret0, frame0 = cap0.read()
            ret1, frame1 = cap1.read()

            if not ret0 or not ret1:
                print('Cameras not returning video data. Exiting...')
                quit()
        else: 
            ret, frame = cap0.read()
            if ret == False:
                # if no video data is received, can't calibrate the camera, so exit.
                print("No video data received from camera. Exiting...")
                quit()

            if arducam_utils.convert2rgb == 0:
                w = cap0.get(cv.CAP_PROP_FRAME_WIDTH)
                h = cap0.get(cv.CAP_PROP_FRAME_HEIGHT)
                frame = frame.reshape(int(h), int(w))

            frame = arducam_utils.convert(frame)
            frame = resize(frame, 1280.0)

            frame0 = frame[0:frame.shape[0], 0:(frame.shape[1]//2)]
            frame1 = frame[0:frame.shape[0], (frame.shape[1]//2):frame.shape[1]]
           


        # follow RGB colors to indicate XYZ axes respectively
        colors = [(0, 0, 255), (0, 255, 0), (255, 0, 0)]
        # draw projections to camera0
        origin = tuple(pixel_points_camera0[0].astype(np.int32))
        for col, _p in zip(colors, pixel_points_camera0[1:]):
            _p = tuple(_p.astype(np.int32))
            cv.line(frame0, origin, _p, col, 2)

        # draw projections to camera1
        origin = tuple(pixel_points_camera1[0].astype(np.int32))
        for col, _p in zip(colors, pixel_points_camera1[1:]):
            _p = tuple(_p.astype(np.int32))
            cv.line(frame1, origin, _p, col, 2)

        cv.imshow('frame0', frame0)
        cv.imshow('frame1', frame1)

        k = cv.waitKey(1)
        if k == 27:
            break

    cv.destroyAllWindows()


def get_world_space_origin(cmtx, dist, img_path):

    frame = cv.imread(img_path, 1)

    # calibration pattern settings
    rows = calibration_settings['checkerboard_rows']
    columns = calibration_settings['checkerboard_columns']
    world_scaling = calibration_settings['checkerboard_box_size_scale']

    # coordinates of squares in the checkerboard world space
    objp = np.zeros((rows*columns, 3), np.float32)
    objp[:, :2] = np.mgrid[0:rows, 0:columns].T.reshape(-1, 2)
    objp = world_scaling * objp

    #print("obj", objp)
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    ret, corners = cv.findChessboardCorners(gray, (rows, columns), None)
    # print(corners)
    cv.drawChessboardCorners(frame, (rows, columns), corners, ret)
    cv.putText(frame, "If you don't see detected points, try with a different image",
               (50, 50), cv.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 1)
    cv.imshow('img', frame)
    cv.waitKey(0)

    ret, rvec, tvec = cv.solvePnP(objp, corners, cmtx, dist)
    # rvec is Rotation matrix in Rodrigues vector form
    R, _ = cv.Rodrigues(rvec)
    print("R:", R)
    print("t:", tvec)

    return R, tvec


def get_cam1_to_world_transforms(cmtx0, dist0, R_W0, T_W0,
                                 cmtx1, dist1, R_01, T_01,
                                 image_path0,
                                 image_path1):

    frame0 = cv.imread(image_path0, 1)
    frame1 = cv.imread(image_path1, 1)

    unitv_points = 5 * np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0],
                                [0, 0, 1]], dtype='float32').reshape((4, 1, 3))
    # axes colors are RGB format to indicate XYZ axes.
    colors = [(0, 0, 255), (0, 255, 0), (255, 0, 0)]

    # project origin points to frame 0
    points, _ = cv.projectPoints(unitv_points, R_W0, T_W0, cmtx0, dist0)
    points = points.reshape((4, 2)).astype(np.int32)
    origin = tuple(points[0])
    for col, _p in zip(colors, points[1:]):
        _p = tuple(_p.astype(np.int32))
        cv.line(frame0, origin, _p, col, 2)

    # project origin points to frame1
    R_W1 = R_01 @ R_W0
    T_W1 = R_01 @ T_W0 + T_01
    points, _ = cv.projectPoints(unitv_points, R_W1, T_W1, cmtx1, dist1)
    points = points.reshape((4, 2)).astype(np.int32)
    origin = tuple(points[0])
    for col, _p in zip(colors, points[1:]):
        _p = tuple(_p.astype(np.int32))
        cv.line(frame1, origin, _p, col, 2)

    cv.imshow('frame0', frame0)
    cv.imshow('frame1', frame1)
    cv.waitKey(0)

    return R_W1, T_W1


def save_extrinsic_calibration_parameters(R0, T0, R1, T1, prefix=''):

    # create folder if it does not exist
    if not os.path.exists('camera_parameters'):
        os.mkdir('camera_parameters')

    camera0_rot_trans_filename = os.path.join(
        'camera_parameters', prefix + 'camera0_rot_trans.dat')
    outf = open(camera0_rot_trans_filename, 'w')

    outf.write('R:\n')
    for l in R0:
        for en in l:
            outf.write(str(en) + ' ')
        outf.write('\n')

    outf.write('T:\n')
    for l in T0:
        for en in l:
            outf.write(str(en) + ' ')
        outf.write('\n')
    outf.close()

    # R1 and T1 are just stereo calibration returned values
    camera1_rot_trans_filename = os.path.join(
        'camera_parameters', prefix + 'camera1_rot_trans.dat')
    outf = open(camera1_rot_trans_filename, 'w')

    outf.write('R:\n')
    for l in R1:
        for en in l:
            outf.write(str(en) + ' ')
        outf.write('\n')

    outf.write('T:\n')
    for l in T1:
        for en in l:
            outf.write(str(en) + ' ')
        outf.write('\n')
    outf.close()

    return R0, T0, R1, T1


if __name__ == '__main__':

    if len(sys.argv) != 2:
        print(
            'Call with settings filename: "python3 Stereo_Calibration.py calibration_settings.yaml"')
        quit()

    # Open and parse the settings file
    parse_calibration_settings_file(sys.argv[1])

    single_source = calibration_settings['single_source']

    """Step1. Save calibration frames for single cameras"""
    save_frames_single_camera(single_source,'camera0') #save frames for camera0
    save_frames_single_camera(single_source,'camera1') #save frames for camera1

    """Step2. Obtain camera intrinsic matrices and save them"""
    # camera0 intrinsics
    images_prefix = os.path.join('frames', 'camera0*')
    cmtx0, dist0 = calibrate_camera_for_intrinsic_parameters(images_prefix)
    # this will write cmtx and dist to disk
    save_camera_intrinsics(cmtx0, dist0, 'camera0')
    # camera1 intrinsics
    images_prefix = os.path.join('frames', 'camera1*')
    cmtx1, dist1 = calibrate_camera_for_intrinsic_parameters(images_prefix)
    # this will write cmtx and dist to disk
    save_camera_intrinsics(cmtx1, dist1, 'camera1')

    cmtx0, dist0 = read_camera_parameters(0)
    cmtx1, dist1 = read_camera_parameters(1)

    """Step3. Save calibration frames for both cameras simultaneously"""
    save_frames_two_cams(single_source,'camera0', 'camera1') #save simultaneous frames

    """Step4. Use paired calibration pattern frames to obtain camera0 to camera1 rotation and translation"""
    frames_prefix_c0 = os.path.join('frames_pair', 'camera0*')
    frames_prefix_c1 = os.path.join('frames_pair', 'camera1*')
    R, T = stereo_calibrate(cmtx0, dist0, cmtx1, dist1,
                            frames_prefix_c0, frames_prefix_c1)

    """Step5. Save calibration data where camera0 defines the world space origin."""
    # camera0 rotation and translation is identity matrix and zeros vector
    R0 = np.eye(3, dtype=np.float32)
    T0 = np.array([0., 0., 0.]).reshape((3, 1))

    save_extrinsic_calibration_parameters(
       R0, T0, R, T)  # this will write R and T to disk

    R1 = R
    T1 = T  # to avoid confusion, camera1 R and T are labeled R1 and T1
    # check your calibration makes sense
    camera0_data = [cmtx0, dist0, R0, T0]
    camera1_data = [cmtx1, dist1, R1, T1]
    check_calibration(single_source,'camera0', camera0_data, 'camera1',
                      camera1_data, _zshift=60.)

    """Optional. Define a different origin point and save the calibration data"""
    # #get the world to camera0 rotation and translation
    # R1 = R
    # T1 = T

    # R_W0, T_W0 = get_world_space_origin(cmtx0, dist0, os.path.join('frames_pair', 'camera0_4.png'))
    # # # #get rotation and translation from world directly to camera1
    # R_W1, T_W1 = get_cam1_to_world_transforms(cmtx0, dist0, R_W0, T_W0,
    #                                            cmtx1, dist1, R1, T1,
    #                                            os.path.join('frames_pair', 'camera0_4.png'),
    #                                            os.path.join('frames_pair', 'camera1_4.png'),)

    # #save rotation and translation parameters to disk
    # save_extrinsic_calibration_parameters(R_W0, T_W0, R_W1, T_W1, prefix = 'world_to_') #this will write R and T to disk
