# importing OpenCV library 
import cv2
import time

cam_port = 1
vid = cv2.VideoCapture(cam_port) 
  
for i in range(1000):
    ret, frame = vid.read() 
    
    cv2.imshow('chessboard', frame) 

    cv2.imwrite(f'Chessboard_{i}.jpg', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    time.sleep(0.2) # pause for 0.2s for testing
  
vid.release() # Remove the vid object
cv2.destroyAllWindows() 