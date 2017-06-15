# import the necessary packages
import cv2
 
# initialize the list of reference points and boolean indicating
# whether cropping is being performed or not
refPt = []
active = True
image = []

def set_image(img):
       global image
       image =img
 
def get_line(event, x, y, flags, param):
       # grab references to the global variables
       global refPt, cropping,image
        
       # if the left mouse button was clicked, record the starting
       # (x, y) coordinates and indicate that cropping is being
       # performed
       if event == cv2.EVENT_LBUTTONDOWN:
              refPt = [(x, y)]
              active = True
              
       # check to see if the left mouse button was released
       elif event == cv2.EVENT_LBUTTONUP:
              # record the ending (x, y) coordinates and indicate that
              # the cropping operation is finished
              refPt.append((x, y))
              active  = False

              # draw a rectangle around the region of interest
              cv2.line(image, refPt[0], refPt[1], (0, 255, 0), 2)
              #cv2.imshow("image", image)
