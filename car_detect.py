import cv2
print (cv2.__version__)
import skvideo.io
import skvideo.datasets
import numpy as np
import utils
import math

MIN_THRESHOLD = 30 #pixels

# Trained XML classifiers describes some features of some object we want to detect
car_cascade = cv2.CascadeClassifier('/home/pi/Documents/cars.xml')
frames = skvideo.io.vread("/home/pi/Downloads/video.avi")

# Create background substraction
fgbg = cv2.createBackgroundSubtractorMOG2()

# Mouse handlers
frame_one = frames[0]
frame_two = frames[0]
height = frame_one.shape[1]
utils.set_image(frame_one)
cv2.namedWindow("frame_one")
cv2.setMouseCallback("frame_one", utils.get_line,0)
left_line = []
right_line = []

print("draw leftmost boundary")
while True:
    # display the image and wait for a keypress
    cv2.imshow("frame_one", frame_one)
    
    left_line = utils.refPt
    if cv2.waitKey(33) == 27:
        break

print("draw rightmost boundary")
while True:
    # display the image and wait for a keypress
    cv2.imshow("frame_one", frame_one)
    
    right_line = utils.refPt
    if cv2.waitKey(33) == 27:
        break

#draw boundary detection
def get_intersection(x1,y1,m1,x2,y2,m2):
        print str(x1)+","+str(y1)+","+str(m1)+","+str(x2)+","+str(y2) + "," +str(m2)
        x_delta = (y2+m2*x2-y1-m2*x1)/(m2+m1)
        return [x1+x_delta,y1+m1*x_delta]
print left_line
print right_line
slope_one = (float(left_line[1][1]) - float(left_line[0][1]))/(left_line[1][0]-left_line[0][0]) 
slope_two = (float(right_line[1][1]) - float(right_line[0][1]))/(right_line[1][0]-right_line[0][0]) 
pt1 = get_intersection(float(left_line[0][0]),float(left_line[0][1]),slope_one,0,0,0)
pt2 = get_intersection(float(left_line[0][0]),float(left_line[0][1]),slope_one,0,height,0)
pt3 = get_intersection(float(right_line[0][0]),float(right_line[0][1]),slope_two,0,0,0)
pt4 = get_intersection(float(right_line[0][0]),float(right_line[0][1]),slope_two,0,height,0)
cv2.line(frame_two, (int(round(pt1[0])),int(round(pt1[1]))),(int(round(pt2[0])),int(round(pt2[1]))), (0, 0, 255), 2)
cv2.line(frame_two, (int(round(pt3[0])),int(round(pt3[1]))),(int(round(pt4[0])),int(round(pt4[1]))), (255, 0, 0), 2)
cv2.imshow("frame_two", frame_two)
cv2.waitKey(100000)


print left_line
print right_line

quit()

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
        #bgmask = fgbg.apply(gray)
     
        # To draw a rectangle in each cars
    for (x,y,w,h) in cars:
        cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)

    #match trajectory with next point        
    delete_trajectories = []
    found_cars = []
    for i in range(0,len(trajectories)):
        trajectory = trajectories[i]
        last_coord = trajectory[len(trajectory)-1]
        min_dist = 1000
        next_point = None
        for j in range(0,len(cars)):
            car = cars[j]
            car_x = car[0] + car[2]/2
            car_y = car[1] + car[3]/2
            dist = math.sqrt((last_coord[1]-car_y)**2 + (last_coord[0]-car_x)**2)
            if dist < min_dist and dist < MIN_THRESHOLD:
                min_dist = dist
                next_point = car
                next_ind = j
        found_cars.append(next_ind)
            
        if next_point is not None:
            (x,y,w,h) = next_point
            trajectories[i].append([x+w/2,y+h/2])
            timeout[i] = 5
        else:
            if timeout[i]==0:
                delete_trajectories.append(i)
            else:
                timeout[i]-=1
                            
    #delete trajectories
    sorted_delete = sorted(delete_trajectories,reverse=True)
    for i in sorted_delete:
            trajectories = trajectories[1:i-1] + trajectories[i+1:len(trajectories)]
            timeout = timeout[1:i-1] + timeout[i+1:len(timeout)]
            
    #insert new cars
    for j in range(0,len(cars)):
        if j not in found_cars:
            (x,y,w,h) = cars[j]
            trajectories.append([[x+w/2,y+h/2]])
            timeout.append(5)

    #print trajectories
    print len(trajectories)
    for trajectory in trajectories:
        if len(trajectory) > 1:
            for i in range(1,len(trajectory)):
                cv2.line(frame,(trajectory[i-1][0],trajectory[i-1][1]),(trajectory[i][0],trajectory[i][1]),(255,0,0),3)
    cv2.imshow("video",frame)

    if cv2.waitKey(33) == 27:
        break

cv2.destroyAllWindows()
