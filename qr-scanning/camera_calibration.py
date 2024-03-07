# Code from: https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
import numpy as np
import cv2 as cv
import glob

# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
images = glob.glob('*.jpg') # collect all images in the current folder that arre .jpg type

for fname in images:

    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY) # convert the colour image into greyscale
    
    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, (7,6), None)
    
    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)
        corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        cv.drawChessboardCorners(img, (7,6), corners2, ret)

        # Save the new pattern onto the original image
        cv.imwrite(f'pattern_{fname}',img)
        cv.waitKey(1)
cv.destroyAllWindows()

# Calibrate the camera to obtain the important camera intrinsic property matrix and distortion parameters
ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# Write the camera matrix and distortion parameters to a .dat file on the same directory as this program:
filename = 'cam_properties.dat'

f = open(filename, 'w') # Open file

# Line by line add the data
for i in range(6):
    if i == 0 :
        f.write('Intrinsic: \n')
    elif i < 4:
        f.write(str(mtx[i-1]) + ' \n')
    elif i == 4:
        f.write('Distortion: \n')
    else:
        f.write(str(dist))

f.close() # Close file