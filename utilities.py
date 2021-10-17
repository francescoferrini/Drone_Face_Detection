'''
            Python file with some funcions useful for the initialization of the drone,
            for the detection of faces and for the tracking
'''

from djitellopy import Tello
import cv2
import numpy as np

''' With the droneInitialization function we created a Tello object and we initialized it '''
def droneInitialization():
    t = Tello()
    ''' Connection to the drone '''
    t.connect()
    ''' Set all velocities to 0 '''
    t.for_back_velocity = 0
    t.left_right_velocity = 0
    t.up_down_velocity = 0
    t.yaw_velocity = 0
    t.speed = 0
    print(t.get_battery())
    ''' If the video is already turned on we swith off and then we switch on'''
    t.streamoff()
    t.streamon()
    return t

''' With the getFrame function we are able to take images from the drone '''
def getFrame(t, w=360, h=240):
    ''' Get the frame '''
    myFrame = t.get_frame_read()
    myFrame = myFrame.frame
    ''' Change the seze of the frame'''
    image = cv2.resize(myFrame, (w, h))
    ''' Return the image '''
    return image

''' With the findFace function we are able to detect the faces in the image just passed to the function '''
def findFace(image):
    ''' We used the haarcascade classifier in order to perform the Viola Jones algorithm '''
    faceCascade = cv2.CascadeClassifier('haarcascade_frontalface_alt2.xml')
    ''' Gray image '''
    imgGray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    ''' We used the classifier to the gray image with 2 parameters:
        scaleFactor -> specify how much the image size is reduced at each image scale
        minNeighbours -> specify how many neighbours each candidate rectangle should have to retain it
    '''
    faces = faceCascade.detectMultiScale(imgGray, scaleFactor=1.1, minNeighbors=6)

    ''' Store the center points of all the images the drone finds. We do it because we want that the drone
        follow the image nearest to the camera '''
    myFaceListC = []
    ''' Store the area '''
    myFaceListArea = []

    ''' Draw the rectangle on the face in the image. x and y are our starting position,
        w is the width and h is the height'''
    for (x, y, w, h) in faces:
        ''' Draw the rectange on the face'''
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 2)
        '''Center x and center y'''
        cx = x + w // 2
        cy = y + h // 2
        area = w * h
        myFaceListArea.append(area)
        myFaceListC.append([cx, cy])

    ''' If there is one face or there are more than one we save the index of the one with the biggest area '''
    if len(myFaceListArea) != 0:
        i = myFaceListArea.index(max(myFaceListArea))
        ''' Return the image and the information on the face (coordinates of the center and area) '''
        return image, [myFaceListC[i], myFaceListArea[i]]
    else:
        ''' Else we only return the image and the drone doesn't move '''
        return image, [[0, 0], 0]

''' With the trackFace function we are able to track a face sending information about it to the PID controller'''
def trackFace(t, coord, w, pid, prevError):
    ''' Before we find the error between the actual value and where we should be coord[0][0] is cx'''
    error = coord[0][0] - w // 2
    ''' We want control the speed and the pid help us to avoid oscillations during the transition '''
    speed = pid[0] * error + pid[1] * (error - prevError)
    ''' We use the clip method in order to keep the speed between -100 and 100 '''
    speed = int(np.clip(speed, -100, 100))

    ''' If there is an image...'''
    if coord[0][0] != 0:
        '''...we change the rotation speed '''
        t.yaw_velocity = speed
    else:
        t.for_back_velocity = 0
        t.left_right_velocity = 0
        t.up_down_velocity = 0
        t.yaw_velocity = 0
        error = 0

    ''' AFter we have setted the values, we have to send them to the drone'''
    if t.send_rc_control:
        t.send_rc_control(t.left_right_velocity,
                                t.for_back_velocity,
                                t.up_down_velocity,
                                t.yaw_velocity)
    return error