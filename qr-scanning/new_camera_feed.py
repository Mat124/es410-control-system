# This code is taken inspiration from: https://github.com/TemugeB/QR_code_orientation_OpenCV/tree/main
import cv2 as cv
import numpy as np
from pyzbar.pyzbar import decode

# Obtain the camera properties
def read_camera_parameters(filepath = 'Calibration/cam_properties.dat'):

    # Open the existing .dat file containing the calibrated camera properties in read mode
    f = open(filepath, 'r')

    # Initialise empty arrays (cmtx, dist) for the camera matrix and distortion parameters respectively
    cmtx = []
    dist = []

    line = f.readline() # ignore the line that says 'Intrinsic'
    # Look at the camera matrix lies 
    for _ in range(3):
        line = f.readline().split() # split the individual numbers in each line under 'Intrinsic' to make a list of 3 elements 
        line = [float(i) for i in line] # convert the split elements from string type into float type and put it inside an array
        cmtx.append(line) # add this array to the top of cmtx

    line = f.readline() # ignore the line that says 'distortion'
    line = f.readline().split()
    line = [float(i) for i in line]
    dist.append(line)

    return np.array(cmtx), np.array(dist)

def get_qr_coords(cmtx, dist, points):

    # Selected coordinate points for each corner of QR code
    qrEdges = np.array([[0,0,0],
                         [0,1,0],
                         [1,1,0],
                         [1,0,0]], dtype = 'float32').reshape((4,1,3)) # reshaped into a 4x1x3 numpy array

    # Determine the orientation of QR code coordinate system with respect to camera coordinate system using Perspective-n-Point Pose computation
    ret, rmtx, tvec = cv.solvePnP(qrEdges, points, cmtx, dist) # rmtx is the 3x3 rotation matrix and tvec is the 3x1 translation vector

    #Define unit xyz axes. These are then projected to camera view using the rotation matrix and translation vector.
    unitvPoints = np.array([[0,0,0], [1,0,0], [0,1,0], [0,0,1]], dtype = 'float32').reshape((4,1,3))
    if ret:
        points, jac = cv.projectPoints(unitvPoints, rmtx, tvec, cmtx, dist)
        return points, rmtx, tvec

    #return empty arrays if rotation and translation values not found
    else: return [], [], []

# Function to display the image with orientation axes
def show_axes(cmtx, dist, source):
    robotName = ''
    # Define fonts and sizes for text on the display
    font = cv.FONT_HERSHEY_SIMPLEX # Defining font type
    fontScale = 1.1 # Defining font size
    fontColour = (255,255,255) # Defining text colour
    fontThickness = 2 # Defining thickness of text

    # Create a videocapure object using the 'source' defined camera
    cap = cv.VideoCapture(source)

    # Initialise the qr code detector object
    qr = cv.QRCodeDetector()

    while True:

        ret, frame = cap.read() # Obtain the frame information 
        if ret == False: 
            break

        capDecoded = decode(frame) # Decode the frame using pyzbar
        retQr, points = qr.detect(frame) # detect the qr code in the frame

        if retQr:
            axisPoints, rvec, tvec = get_qr_coords(cmtx, dist, points)

            #BGR color format
            colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (0,0,0)]

            #check axes points are projected to camera view.
            if len(axisPoints) > 0:
                axisPoints = axisPoints.reshape((4,2))

                origin = (int(axisPoints[0][0]),int(axisPoints[0][1]) )

                for p, c in zip(axisPoints[1:], colors[:3]):
                    p = (int(p[0]), int(p[1]))

                    #Sometimes qr detector will make a mistake and projected point will overflow integer value. We skip these cases. 
                    if origin[0] > 5*frame.shape[1] or origin[1] > 5*frame.shape[1]:
                        break
                    if p[0] > 5*frame.shape[1] or p[1] > 5*frame.shape[1]:
                        break

                    cv.line(frame, origin, p, c, 5)

        # Obtain the text assigned to the QR code
        for barcode in capDecoded:
            robotName = barcode.data.decode('utf-8')

        txtFrame = cv.putText(frame, robotName, (0,30), font, fontScale, fontColour, fontThickness) # Make new frame with the text of the robot 
        cv.imshow('', txtFrame) # display the frame with the name of the robot in the top left corner of the frame

        k = cv.waitKey(20)
        # If the ESC key is pressed then leave the while loop (#27 is the ESC key)
        if k == 27: 
            break

    cap.release()
    cv.destroyAllWindows()

if __name__ == '__main__':

    #read camera intrinsic parameters.
    cmtx, dist = read_camera_parameters()

    camNum = 1 # Which camera to use 0 is internal, and other is external

    show_axes(cmtx, dist, camNum)