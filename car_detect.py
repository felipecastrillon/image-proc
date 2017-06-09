import cv2
print (cv2.__version__)
import skvideo.io
import skvideo.datasets
import numpy as np
import utils
import math

MIN_THRESHOLD = 15 #pixels

# Trained XML classifiers describes some features of some object we want to detect
car_cascade = cv2.CascadeClassifier('/home/pi/Documents/cars.xml')

frames = skvideo.io.vread("/home/pi/Downloads/video1.avi")

# Create background substraction
fgbg = cv2.createBackgroundSubtractorMOG2()

# Mouse handlers
image = frames[0]
utils.set_image(image)
cv2.namedWindow("image")
#cv2.setMouseCallback("image", utils.get_line,0)

#while True:
        # display the image and wait for a keypress
#	cv2.imshow("image", image)
#	key = cv2.waitKey(1) & 0xFF
 
	# if the 'c' key is pressed, break from the loop
#	if key == ord("c"):
#		break

trajectories =[]
timeout = []

for frame in frames:
       
        if (type(frame) == type(None)):
                break

         # convert to gray scale of each frames
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        #gray = cv2.GaussianBlur(gray, (21, 21), 0)

        #bgmask = fgbg.apply(gray)
        

        # dilate the thresholded image to fill in holes, then find contours
	# on thresholded image
	#thresh = cv2.threshold(bgmask, 25, 255, cv2.THRESH_BINARY)[1]
	#thresh = cv2.dilate(thresh, None, iterations=2)
	#(_,cnts, _) = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
#		cv2.CHAIN_APPROX_SIMPLE)

        # loop over the contours
	#for c in cnts:
                # if the contour is too small, ignore it
	#	if cv2.contourArea(c) < 20:
	#		continue
 
		# compute the bounding box for the contour, draw it on the frame,
		# and update the text
	#	(x, y, w, h) = cv2.boundingRect(c)
		#cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # Detects cars of different sizes in the input image
        cars = car_cascade.detectMultiScale(gray, 1.1, 1)
        print "STEP"
        print "--------"
        print cars
        print "--------"
        #bgmask = fgbg.apply(gray)
     
        # To draw a rectangle in each cars
        for (x,y,w,h) in cars:
                cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)

        delete_trajectories = []
        found_trajectories = []
        for i in range(0,len(trajectories)):
                trajectory = trajectories[i]
                last_coord = trajectory[len(trajectory)-1]
                min_dist = 1000
                next_point = None
                for j in range(0,len(cars)):
                       car = cars[j]
                       dist = math.sqrt((last_coord[1]-car[1])**2 + (last_coord[0]-car[0])**2)
                       if dist < min_dist and dist < MIN_THRESHOLD:
                              min_dist = dist
                              next_point = car
                              #found_trajectories.append(j)
                if next_point is not None:
                       (x,y,w,h) = next_point
                       trajectories[i].append([x+w/2,y+h/2])
                       print trajectories[i]
                #else:
                #       if timeout[i]==0:
                #              delete_trajectories.append(i)
                #       else:
                #              timeout[i]-=1   

        for j in range(0,len(cars)):
                if j not in found_trajectories:
                       (x,y,w,h) = cars[j]
                       trajectories.append([[x+w/2,y+h/2]])
                       timeout.append(5)

        print "--------"
        print trajectories
        print "--------"
        for trajectory in trajectories:
                 print trajectory
                 if len(trajectory) > 1:
                        for point in trajectory:
                               cv2.circle(frame,(point[0],point[1]),2,(255,0,0),2)
              
        cv2.imshow("video",frame)

        if cv2.waitKey(33) == 27:
                break

cv2.destroyAllWindows()
