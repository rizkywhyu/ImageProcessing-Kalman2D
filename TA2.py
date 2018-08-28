# This code was based on rosebrock's code.
# http://www.computervisiononline.com/blog/tutorial-using-camshift-track-objects-video
# And we used simondlevy's code. 
# https://github.com/simondlevy/OpenCV-Python-Hacks/tree/master/Kalman2D
#import the necessary packages
import serial
import string
import numpy as np
import argparse
import cv2
import time
import picamera
import picamera.array
import datetime
import os
import imutils
from scipy.spatial import distance as dist
from imutils import perspective
from imutils import contours
from matplotlib import pyplot as plt
from datetime import datetime
from datetime import time
from datetime import timedelta
from PIL import Image
from sys import exit
from kalman2d import Kalman2D



frame = None
#firstframe = None
roiPts = []
inputMode = False
Width = 480
Height = 480

wl = Width*4.5/10

wr = Width*5.5/10

ht = Height*4.5/10

hb = Height*5.5/10

targetBox = np.array([[wl,ht], [wr,ht], [wr,hb], [wl,hb]])

ser = serial.Serial('/dev/ttyAMA0', baudrate=9600, timeout=1.0)
ser.close()
ser.open()
ser.flushInput()
ser.flushOutput()
#Titik Tengah 

class CenterInfo(object):


        def __init__(self): #Inisialisasi tengah

                self.x, self.y = -1, -1

        def __str__(self): #menampilkan titik tengah

                return '%4d %4d' % (self.x, self.y)


#memilih roibox dari mouse

def selectROI(event, x, y, flags, param):

    global frame, roiPts, inputMode

    if inputMode and event == cv2.EVENT_LBUTTONDOWN and len(roiPts) < 4:

        roiPts.append((x,y))

        cv2.circle(frame,(x,y),4,(0,255,0),2)

        cv2.imshow("frame",frame)

def midpoint(ptA, ptB):
        return ((ptA[0] + ptB[0]) * 0.5, (ptA[1] + ptB[1]) * 0.5)


def main():
	lower = [17, 15, 100]
	upper = [50, 56, 200]
	lower = np.array(lower, dtype = "uint8")
	upper = np.array(upper, dtype = "uint8")

        global frame, roiPts, inputMode, roiBoxWidth, roiBoxHeight

        cnt = 0    #menghitung roibox dari kalmanfilter 

        centerX = 0
        
        centerY = 0
	
        toggle = True 

        flag = False 


        kalman2d = Kalman2D() 


        cv2.namedWindow("frame")

        cv2.setMouseCallback("frame",selectROI)


        termination = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,10,3) #kondisi terminasi camshift

        roiBox = None

        time2 = datetime.now() #<<<check time>>>


        serial_flag_bit = 1 #Sending data flag

        delta = timedelta(seconds = 1) 

        #menggunakan capture_continous

        with picamera.PiCamera() as camera:

            stream = picamera.array.PiRGBArray(camera)

            camera.resolution = (Width,Height) #mengatur resolusi

            time3 = datetime.now() 

            try:

                for foo in enumerate(camera.capture_continuous(stream,'bgr',use_video_port = True)): #menangkap dari capture_continuous

                    time1 = datetime.now() 

                    # print(time1 - time2) #periode waktu pada 1 cycle terhadap loop
		
                    time2 = time1 

                    frame = stream.array #menyimpan array gambar ke variabel
                    mask = cv2.inRange(frame, lower, upper)
                    output = cv2.bitwise_and(frame, frame, mask = mask)

                    gray = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)
                    gray = cv2.GaussianBlur(gray, (7, 7), 0)

                    edged = cv2.Canny(gray, 50, 100)
                    edged = cv2.dilate(edged, None, iterations=1)
                    edged = cv2.erode(edged, None, iterations=1)
                        

                    stream.seek(0) #inisialisasi stream untuk iterasi selanjutnya

                    stream.truncate() 

                    if roiBox is not None:# definisi roiBox

                        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) #merubah warna gambar RGB ke HSV
                        # print ('hsv: ', hsv)
                        backProj = cv2.calcBackProject([hsv],[0],roiHist, [0,180], 1) #membuat backprojection
                        # print ('backproj', backProj)
                        if roiBoxWidth > 0 and roiBoxHeight > 0: #Cek camshift jika gagal

                            (r,roiBox) = cv2.CamShift(backProj, roiBox, termination) #camshift

                            #r is set of 4points about roiBox

                            #roiBox is an array, consist of x and y of topLeft, and box's width and height

                            roiBoxWidth = roiBox[2]

                            roiBoxHeight = roiBox[3]

                            serial_flag_bit = 1

                        else :

                            #Init roiBox ketika camshift gagal

                            # print "roiBox init!!!!!!!!!!!!!!!!"

                            tl[0] = 1

                            tl[1] = 1

                            br[0] = Width -1

                            br[1] = Height -1

                            roiBox = (tl[0], tl[1], br[0], br[1])                                                

                            #Camshift

                            (r,roiBox) = cv2.CamShift(backProj, roiBox, termination)

                            roiBoxWidth = roiBox[2]

                            roiBoxHeight = roiBox[3]

                            serial_flag_bit = 0

                            time3 = datetime.now()


                        #transform r ke pts ke draw Box dengan polylines

                        pts = np.int0(cv2.cv.BoxPoints(r))
                        

                        #Draw roiBox

                        cv2.polylines(frame,[pts],True, (0,255,0),2)
                        

                        #menghitung center x,y

                        centerX = (pts[0][0] + pts[2][0])/2
                        print ('Center X', centerX)
                       
                        centerY = (pts[0][1] + pts[2][1])/2
                        
                        

                        #Update x,y to kalman filter dan mendapatkan estimasi x,y

                        CenterInfo.x = centerX
                       
                        CenterInfo.y = centerY
                        
                        

                        #mengirim center x ke arduino

                        if CenterInfo.x / 10 == 0:

                            tempCenterX = '00' + str(CenterInfo.x)

                        elif CenterInfo.x / 100 == 0:

                            tempCenterX = '0' + str(CenterInfo.x)

                        else:

                            tempCenterX = str(CenterInfo.x)

                        

                        if CenterInfo.y / 10 == 0:

                            tempCenterY = '00' + str(CenterInfo.y)

                        elif CenterInfo.y / 100 == 0:

                            tempCenterY = '0' + str(CenterInfo.y)

                        else:

                            tempCenterY = str(CenterInfo.y)

                        # centerData = str(int(centerX)) 
                        # print ("Center Data", centerData)


                        #check waktu dan flag saat mengirim data ke arduino

                        if datetime.now() > time3 + delta :

                            if serial_flag_bit == 1:

                                ser.write(tempCenterX )
                                # print ser.write(tempCenterX)

                        

                        #Update Kalman

                        kalman2d.update(CenterInfo.x, CenterInfo.y)

                        estimated = [int (c) for c in kalman2d.getEstimate()]


                        estimatedX = estimated[0]
                        print ('Estimasi x', estimatedX)
                        estimatedY = estimated[1]

                        

                        #menghitung delta x,y

                        deltaX = estimatedX - centerX
                        
                        deltaY = estimatedY - centerY

                        

                        #Apply new roiBox from kalmanfilter

                        if cnt > 1:

                            roiBox = (roiBox[0]+deltaX, roiBox[1]+deltaY, br[0], br[1])

                            cnt = 0

                        

                        #Draw estimated center x,y from kalman filter for test

                        #cv2.circle(frame,(estimatedX,estimatedY), 4, (0, 255, 255),2)

                        

                        #merubah warna target ketika berada di box

                        #if wl < centerX and wr > centerX and centerY < hb and centerY > ht :

                            #cv2.circle(frame,(centerX,centerY), 4, (255,0,0),2)

                            #flag = False

                        #else :

                            #cv2.circle(frame,(centerX,centerY), 4, (0,255,0),2)

                            #flag = True


                        cnt = cnt+1    #menghitung box baru dari kalman filter                

                        

                        #Draw kalman top left point for test

                        #cv2.circle(frame,(roiBox[0],roiBox[1]), 4, (0,0,255),2)

                    #Draw target box

                    cv2.circle(frame,(Width/2,Height/2) , 4, (255,255,255),2)

                    cv2.polylines(frame, np.int32([targetBox]), 1, (255,255,255))

                    # print(len(cnts))
                    colors = ((0, 0, 255), (240, 0, 159), (0, 165, 255), (255, 255, 0), (255, 0, 255))
                    # refObj = None
                    # find contours in the edge map
                    cnts = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    # print("cnts1",cnts)
                    if not cnts[0] == []:
                        cnts = cnts[0] if imutils.is_cv2() else cnts[1]
                        # print("cnts2",cnts)

                        # sort the contours from left-to-right and, then initialize the
                        # distance colors and reference object
                        (cnts, _) = contours.sort_contours(cnts) 
                        for c in cnts:
                            # if the contour is not sufficiently large, ignore it
                            if cv2.contourArea(c) < 100:
                                continue

                            # compute the rotated bounding box of the contour
                            box = cv2.minAreaRect(c)
                            box = cv2.cv.BoxPoints(box) if imutils.is_cv2() else cv2.boxPoints(box)
                            box = np.array(box, dtype="int")

                            # order the points in the contour such that they appear
                            # in top-left, top-right, bottom-right, and bottom-left
                            # order, then draw the outline of the rotated bounding
                            # box
                            box = perspective.order_points(box)

                            # compute the center of the bounding box
                            cX = np.average(box[:, 0])
                            cY = np.average(box[:, 1])
                            (tl, tr, br, bl) = box
                            (tlblX, tlblY) = midpoint(tl, bl)
                            (trbrX, trbrY) = midpoint(tr, br)

                            # compute the Euclidean distance between the midpoints,
                            # then construct the reference object
                            D = dist.euclidean((tlblX, tlblY), (trbrX, trbrY))
                            refObj = (box, (cX, cY), D / 3.5)
                            # print("refObj",refObj)
                            cv2.drawContours(frame, [box.astype("int")], -1, (0, 255, 0), 2)

                            refCoords = np.vstack([refObj[0], refObj[1]])
                            objCoords = np.vstack([pts, (centerX, centerY)])
                            # print("refCoords", refCoords)
                            # print("objCoords",objCoords)
                            
                            # loop over the original points
                            # for ((xA, yA), (xB, yB), color) in zip(refCoords, objCoords, colors):
                            xA, yA = refCoords[-1]
                            xB, yB = objCoords[-1]
                            color = colors[-1]

                            # draw circles corresponding to the current points and
                            # connect them with a line
                            cv2.circle(frame, (int(xA), int(yA)), 5, color, -1)
                            cv2.circle(frame, (int(xB), int(yB)), 5, color, -1)
                            cv2.line(frame, (int(xA), int(yA)), (int(xB), int(yB)),
                                color, 2)

                            # compute the Euclidean distance between the coordinates,
                            # and then convert the distance in pixels to distance in
                            # units
                            D = dist.euclidean((xA, yA), (xB, yB)) / refObj[2]
                            hasil = D/29.5276
                            print("Distance", hasil)
                            if D <= 29.5276:
                                print("Anda Mendekati Area Berbahaya")
                            (mX, mY) = midpoint((xA, yA), (xB, yB))
                            cv2.putText(orig, "{:.1f}in".format(D), (int(mX), int(mY - 10)),cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
                    #menampilkan atau tidak imshow
                    #cnts = []
                    if toggle is True:

                        cv2.imshow("frame",frame) #if you want speed up, delete this line
                        

                    key=cv2.waitKey(1) & 0xFF


                    #Init roiBox

                    if key == ord("i") and len(roiPts) < 4:

                        inputMode = True

                        orig = frame.copy()

                                                

                        #wait for 4th click

                        while len(roiPts) < 4:

                            cv2.imshow("frame",frame)

                            cv2.waitKey(0)

                        

                        #set data from 4 clicks

                        roiPts = np.array(roiPts)

                        s = roiPts.sum(axis=1)

                        tl = roiPts[np.argmin(s)]

                        br = roiPts[np.argmax(s)]


                        #membuat warna histogram dari roi

                        roi = orig[tl[1]:br[1], tl[0]:br[0]]

                        roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
			
                        # if ser.isOpen():
                        #     read_data = ser.read()
                        #     #response  = ser.readline()
                        #     print "Data received : " + read_data

                        # else:
                        #     print "Can not open serial port"

                        

                        roiHist = cv2.calcHist([roi],[0],None,[16],[0,180])

                        roiHist = cv2.normalize(roiHist, roiHist,0,255,cv2.NORM_MINMAX)

                        roiBox = (tl[0], tl[1], br[0], br[1])

                        

                        #mengatur roiBox lebar, tinggi

                        roiBoxWidth = roiBox[2]

                        roiBoxHeight = roiBox[3]

                        

                        #menghitung center x,y

                        CenterInfo.x = tl[0]+br[0]/2

                        CenterInfo.y = tl[1]+br[1]/2


                        #Init x,y to kalman filter and set first roiBox

                        CenterInfo.x = centerX

                        CenterInfo.y = centerY


                        kalman2d.update(CenterInfo.x, CenterInfo.y)

                        estimated = [int (c) for c in kalman2d.getEstimate()]
                        print kalman2d
                        print estimated
                        estimatedX = estimated[0]

                        estimatedY = estimated[1]


                        #Calculate delta x,y

                        deltaX = estimatedX - centerX
                        print ('Delta x2', deltaX)
                        deltaY = estimatedY - centerY


                        #set first roiBox

                        roiBox = (tl[0]+deltaX, tl[1]+deltaY, br[0], br[1])

                    


                    #toggle for imshow

                    elif key == ord("r"):

                        toggle = not toggle

                    #quit

                    elif key == ord("q"):

                        break


            finally:

                cv2.destroyAllWindows()


if __name__== "__main__":

    main()







