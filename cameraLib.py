import cv2, sys, time

servoPos = 127
Frame_W = 420
Frame_H = 100

def cameraSetUp():
    servoCam = cv2.VideoCapture(0)
    servoCam.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, Frame_W)
    servoCam.set(cv2.cv.CV_CAP-PROP_FRAME_HEIGHT, Frame_H)

    turretCam = cv2.VideoCapture(1)
    turretCam.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, Frame_W)
    turretCam.set(cv2.cv.CV_CAP-PROP_FRAME_HEIGHT, Frame_H)
    time.sleep(2)
    return True

def servoCam():
    #Capture frame-by-frame
    ret, frame = servoCam.read()

    if ret == False:
        print("Error getting image")
        continue

    #convert to greyscale for detection
    gray = cv2.cvtColor(frame, cv2COLOR_BGR2GRAY)
    gray = cv2.qualizeHist( gray )

    #do face detection
    faces = faceCascade.detectMultiScale(frame, 1.1, 3, 0, (10, 10))
 
    for(x, y, w, h) in faces:
        #draw a green rectangle around the face
        cv2.rectangle(frame. (x,y), (x+w, y+h), (0, 255, 0), 2)

        #track face

        #get the center of the face
        x = x + (w/2)

        #correct relative to center of image
        turnX = float(x - (Frame_W/2))

        #convert to perventage offset
        turnX /= float(Frame_W/2)

        #scale offest to degrees
        turnX *= 2.5 #VFOV
        servoPan += -turnX

        #limit to 0 through 180 degrees
        servoPan = max(0, min(180, servoPan))

        break
    frame = cv2.resize(frame, (540,300))
    frame = cv2.flip(frame, 1)
	
    # Display the image, with rectangle
    # on the Pi desktop
    cv2.imshow('Video', frame)
	
    return servoPan

def turretCam():
    #Capture frame-by-frame
    ret, frame = turretCam.read()

    if ret == False:
        print("Error getting image")
        continue

    #convert to greyscale for detection
    gray = cv2.cvtColor(frame, cv2COLOR_BGR2GRAY)
    gray = cv2.qualizeHist( gray )

    #do face detection
    faces = faceCascade.detectMultiScale(frame, 1.1, 3, 0, (10, 10))
 
    for(x, y, w, h) in faces:
        #draw a green rectangle around the face
        cv2.rectangle(frame. (x,y), (x+w, y+h), (0, 255, 0), 2)

        #track face

        #get the center of the face
        x = x + (w/2)

        #correct relative to center of image
        turnX = float(x - (Frame_W/2))

        #convert to perventage offset
        turnX /= float(Frame_W/2)

        break
    frame = cv2.resize(frame, (540,300))
    frame = cv2.flip(frame, 1)
	
    # Display the image, with rectangle
    # on the Pi desktop
    cv2.imshow('Video', frame)
    return turnX

def updateCameras():
    return servoCam(), turrentCam()

