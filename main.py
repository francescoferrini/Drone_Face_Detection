'''
            Main python file for the Drone Face Tracking program
            Francesco Ferrini, Wamiq Raza
'''

from utilities import *
import cv2

''' Some variables '''
w, h = 360, 240
''' Small will be the values and smoothest wiil be the movement'''
pid = [0.4, 0.4, 0]
prevError = 0
droneUp = 0  # for no Flight 1   - for flight 0

''' Initialization of the drone'''
t = droneInitialization()

while True:
    if droneUp == 0:
        #t.takeoff()
        droneUp = 1

    ''' Get the frame from our drone '''
    image = getFrame(t, w, h)
    ''' Find the face in the image. We also find the coordinates of the image and send to pid controller'''
    image, coord = findFace(image)
    ''' Track the biggest face '''
    prevError = trackFace(t, coord, w, pid, prevError)
    ''' Show the frame '''
    cv2.imshow('Image', image)
    ''' Command to stop the drone '''
    if cv2.waitKey(1) & 0xFF == ord('q'):
        t.land()
        break