from dronekit import *
from pymavlink import mavutil
import time
from math import *
import cv2
import numpy as np
import imutils.video
from collections import Counter
import webcolors
import operator
import matplotlib.pyplot as plt
import csv
import os
import threading

"The following code is used for detection of a square containing another square within in there is a colour and " \
"alphnumeric character which is needed to be recognition and provide the location of the GPS coordinates of the Targets"


class ContourWithData():
  # member variables ############################################################################
  npaContour = None  # contour
  boundingRect = None  # bounding rect for contour
  intRectX = 0  # bounding rect top left corner x location
  intRectY = 0  # bounding rect top left corner y location
  intRectWidth = 0  # bounding rect width
  intRectHeight = 0  # bounding rect height
  fltArea = 0.0  # area of contour
  intCentreX = 0
  intCentreY = 0

  def calculateRectTopLeftPointAndWidthAndHeight(self):  # calculate bounding rect info
    [intX, intY, intWidth, intHeight] = self.boundingRect
    self.intRectX = intX
    self.intRectY = intY
    self.intCentreX = intX / 2
    self.intCentreY = intY / 2
    self.intRectWidth = intWidth
    self.intRectHeight = intHeight
    self.fltDiagonalSize = math.sqrt((self.intRectWidth ** 2) + (self.intRectHeight ** 2))

  def checkIfContourIsValid(self, height, width):  # this is oversimplified, for a production grade program
    MAX_CONTOUR_AREA = height * width * 0.9
    if MIN_CONTOUR_AREA < self.fltArea < MAX_CONTOUR_AREA:
      return True
    return False

def gpsDistance(lat):
  # Returns approximate distance in meters of 1 degree of latitude and longitude at certain latitude, correct to a few centimetres, using WGS84 spheroid model
  # Also known as length of a degree
  latLength = 111132.92 - 559.82 * cos(radians(lat)) + 1.175 * cos(4 * radians(lat)) - 0.0023 * cos(6 * radians(lat))
  longLength = 111412.84 * cos(radians(lat)) - 93.5 * cos(3 * radians(lat)) + 0.118 * cos(5 * radians(lat))

  return [latLength, longLength]

def GPS(targetPos, heading, lat, long, alt, height_of_target):
  # Gets gps coordinates of target
  # Takes a targetPos array of the centre of the target's position in frame [y,x]
  # takes heading and 3d position of plane at time of image capture
  # outputs array with [lat, lon] of target

  # get image dimensions and centre of image for aircraft position
  imgDimensions = [480, 640]
  imgCentre = [imgDimensions[0] / 2, imgDimensions[1] / 2]

  # Create array for position of plane in 3D space, [lat, long, altitude]
  plane3DPos = [lat, long, alt]

  #Get difference between image centre and target image y,x axis
  targetFromPlane = []
  for i in range(2):
    targetFromPlane.append(abs(targetPos[i] - imgCentre[i]))

    # get (b) 'bearing' of target relative to plane's heading vector
    if targetPos[0] > imgCentre[0]:
      if targetPos[1] > imgCentre[1]:
        b = degrees(atan(targetFromPlane[0] / targetFromPlane[1])) + 90
      elif targetPos[1] < imgCentre[1]:
        b = degrees(atan(targetFromPlane[1] / targetFromPlane[0])) + 180
    if targetPos[0] < imgCentre[0]:
      if targetPos[1] > imgCentre[1]:
        b = degrees(atan(targetFromPlane[1] / targetFromPlane[0]))
      elif targetPos[1] < imgCentre[1]:
        b = degrees(atan(targetFromPlane[0] / targetFromPlane[1])) + 270

    # Get bearing of target from aircraft (relative to North)
    if b < 360 - heading:
      targetBearing = b + heading
    elif b > 360 - heading:
      targetBearing = (b + heading - 360) / 2

    while air == 1:

      # conversion between pixels and actual distance in meters, based on height and lens
      # meters in 1 pixel in image at altitude during image capture
      pixToMeters = (alt * cos(radians(camera_lens_angle)) * 2) / (sqrt(imgDimensions[0] ** 2 + imgDimensions[1] ** 2))

      # Get distance of target from aircraft in meters, in y,x frame components
      distanceComp = []
      for i in range(2):
        distanceComp.append(targetFromPlane[i] * pixToMeters)
      distanceFromTarget = sqrt(distanceComp[1] ** 2 + distanceComp[0] ** 2)

      # Get components of distance to target in m in Lat and Long axis
      alignedDist = [distanceFromTarget * cos(radians(targetBearing)), distanceFromTarget * sin(radians(targetBearing))]

    else:
      # for the RC Rover to get the lat, long and alt in (degree, degree, metre)
      # conversion between pixels and actual distance in meters, based on height and lens
      pixToMeters = 2 * alt / imgDimensions[1]  # meters in 1 pixel in image at altitude during image capture

      # grabs the target new altitude (m)
      new_alt = targetPos[1] * pixToMeters

      # grab the maximum length (m)
      length_max = 2 * alt * cos(radians(camera_lens_angle))

      # Gets the scale of the max altitude of camera and the difference of the target and camera centre (m)
      scale = (2 * alt) / (height_of_target * pixToMeters)

      # distance of target from camera (m)
      distance = length_max / scale

      # Get components of distance to target in m in Lat and Long axis
      alignedDist = distance * cos(radians(targetBearing)), distance * sin(radians(targetBearing))

    # Get distance in meters of 1 degree of lat and long at image capture altitude
    gpsDist = gpsDistance(plane3DPos[0])

    # Get distance between target and rover in degrees latitude and longitude (conversion)
    gpsTargetOffset = []
    for i in range(2):
      gpsTargetOffset.append(alignedDist[i] / gpsDist[i])

    # Get gps coords of target
    targetCoords = []
    for i in range(2):
      targetCoords.append(plane3DPos[i] + gpsTargetOffset[i])

    if air != 1:
      # add the new altitude to the list
      targetCoords.append(new_alt)

    return targetCoords

def solution(counter, marker, distance):
  print("detection of marker", marker, "located")
  print(character(counter, marker, distance) + " is located for marker", marker)
  print(colour(counter, marker, distance) + " is the colour of ground marker", marker)

  # if Static_Test:
  with open('results.csv', 'a') as csvfile: # for testing purposes
    filewriter = csv.writer(csvfile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL, lineterminator='\n')
    filewriter.writerow([str(marker), str(character(counter, marker, distance)), str(colour(counter, marker, distance))])

  if GPS and air == 1:
    # Get middle position within lists outputted by detection function
    middle = int(len(positions) / 2)

    print(GPS(centres[middle], headings[middle], positions[middle][0], positions[middle][1], positions[middle][2],
              height_of_target[middle]) + " latitude and longitude of", marker)
  elif GPS and air != 1:
    # Get middle position within lists outputted by detection function
    middle = int(len(positions) / 2)

    print(GPS(centres[middle], headings[middle], positions[middle][0], positions[middle][1], positions[middle][2],
              height_of_target[middle]) + " latitude, longitude and altitidue of", marker)
  counter = 0
  marker = marker + 1

  return counter, marker

def detection():
  print('Starting detection')

  # Initialising variable
  counter = 0
  marker = 1
  positions = []
  headings = []
  centres = []
  height_of_target = []
  square = 2

  # if Static_Test:
  # cap = cv2.VideoCapture("TestData2.mp4")  # video use

  cap = cv2.VideoCapture(0)
  cap.set(cv2.CAP_PROP_FRAME_WIDTH, 800) #800
  cap.set(3, 800) #800
  cap.set(4, 800) #800
  cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 800)
  cap.set(cv2.CAP_PROP_FPS, 60)

  time.sleep(2)  # allows the camera to start-up
  print('Camera on')
  # Run detection when camera is turn on
  while (cap.isOpened()): # for video use
  # while True:
    # the camera will keep running even after the if statement so it can detect multiple ground marker
    if counter == 0 or start - end < 5:
        if Static_Test:
          distance = input("Distance it was taken")
        #  start - end < 5
        if not Static_Test:
          distance = 1
        ret, frame = cap.read()

        # Gathering data from Pixhawk
        if GPS:
          position = vehicle.location.global_relative_frame
          heading = vehicle.heading
        # end if

        # starting the timer for the length of time it hasn't found a target
        start = time.time()

        # applying image processing
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # converts to gray
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)  # blur the gray image for better edge detection
        edged = cv2.Canny(blurred, 14, 10)  # the lower the value the more detailed it would be

        # find contours in the thresholded image and initialize the
        (contours, _) = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)  # grabs contours

        # outer square
        for c in contours:
          peri = cv2.arcLength(c, True)  # grabs the contours of each points to complete a shape
          # get the approx. points of the actual edges of the corners
          approx = cv2.approxPolyDP(c, 0.01 * peri, True)
          if 4 <= len(approx) <= 6:
            (x, y, w, h) = cv2.boundingRect(approx)  # gets the (x,y) of the top left of the square and the (w,h)
            aspectRatio = w / float(h)  # gets the aspect ratio of the width to height
            area = cv2.contourArea(c)  # grabs the area of the completed square
            hullArea = cv2.contourArea(cv2.convexHull(c))
            solidity = area / float(hullArea)
            keepDims = w > 25 and h > 25
            keepSolidity = solidity > 0.9  # to check if it's near to be an area of a square
            keepAspectRatio = 0.6 <= aspectRatio <= 1.4
            if keepDims and keepSolidity and keepAspectRatio:  # checks if the values are true
              # captures the region of interest with a 5 pixel lesser in all 2D directions
              roi = frame[y:y + h, x:x + w]

              height, width, numchannels = frame.shape

              centre_region = (x + w / 2, y + h / 2)
              if GPS:
                centre_target = (y + h / 2, x + w / 2)

              # grabs the angle for rotation to make the square level
              angle = cv2.minAreaRect(approx)[-1]  # -1 is the angle the rectangle is at

              if 0 == angle:
                angle = angle
              elif -45 > angle > 90:
                angle = -(90 + angle)
              elif -45 > angle:
                angle = 90 + angle
              else:
                angle = angle

              rotated = cv2.getRotationMatrix2D(tuple(centre_region), angle, 1.0)

              imgRotated = cv2.warpAffine(frame, rotated, (width, height)) # width and height was changed

              imgCropped = cv2.getRectSubPix(imgRotated, (w, h), tuple(centre_region))

              HSVCropp = cv2.cvtColor(imgCropped, cv2.COLOR_BGR2HSV)

              if square == 2:
                color = imgCropped[int((h/2)-(h/4)):int((h/2)+(h/4)), int((w/2)-(w/4)):int((w/2)+(w/4))]
              else:
                color = imgCropped

              if Step_detection:
                cv2.imshow("crop", imgCropped)
                cv2.imshow("okay", color)
                print(HSVCropp[int((h / 2) - (h * (6 / 10))), int((w / 2) - (w * (6 / 10)))])

              # # Convert the image to grayscale and turn to outline of  the letter
              # g_rotated = cv2.cvtColor(imgCropped, cv2.COLOR_BGR2GRAY)
              # b_rotated = cv2.GaussianBlur(g_rotated, (5, 5), 0)
              # e_rotated = cv2.Canny(b_rotated, 70, 20)
              #
              # # uses the outline to detect the corners for the cropping of the image
              # (contours, _) = cv2.findContours(e_rotated.copy(), cv2.RETR_LIST,
              #                                  cv2.CHAIN_APPROX_SIMPLE)
              #
              # # inner square detection
              # for cny in contours:
              #   perin = cv2.arcLength(cny, True)
              #   approxny = cv2.approxPolyDP(cny, 0.01 * perin, True)
              #   if 4 <= len(approxny) <= 6:
              #     (xx, yy), (ww, hh), angle = cv2.minAreaRect(approxny)
              #     aspectRatio = ww / float(hh)
              #     keepAspectRatio = 0.7 <= aspectRatio <= 1.3
              #     angle = cv2.minAreaRect(approxny)[-1]
              #     keep_angle = angle == 0, 90, 180, 270, 360
              #     if keepAspectRatio and keep_angle:
              #       (xxx, yyy, www, hhh) = cv2.boundingRect(approxny)
              #       color = imgCropped[yyy:yyy + hhh, xxx:xxx + www]

              # appends the data of the image to the list
              if GPS:
                positions.append([position.lat, position.lon, position.alt])
                headings.append(heading)
                centres.append(centre_target)
                height_of_target.append(h)

              # time that the target has been last seen
              end = time.time()
              time.sleep(0.5)

              # keep count of number of saved images
              counter = counter + 1
              cv2.imwrite("colour%d.png" % counter, color)
              cv2.imwrite('C:/Users/kevin/Desktop/2018-2019/method A/results/{0}_{1}.png'.format(marker, counter), color)
              print("Detected and saved a target")

              if Static_Test:
                # testing purposes
                if not os.path.exists(distance):
                  os.makedirs(distance)
                cv2.imwrite('C:/Users/kevin/Desktop/2018-2019/method A/{0}/results{1}_{2}.png'.format(distance, marker, counter), color)
                cv2.imwrite('C:/Users/kevin/Desktop/2018-2019/method A/{0}/captured{1}_{2}.png'.format(distance, marker, counter), roi)
                cv2.imwrite('C:/Users/kevin/Desktop/2018-2019/method A/{0}/orginal{1}_{2}.png'.format(distance, marker, counter), frame)
              else:
                distance = 0

              if Step_detection:
                cv2.imshow("roi", roi)
                cv2.imshow("cropped", imgCropped)
                cv2.waitKey(0)
              # end if
              if counter == 7:
                counter, marker = solution(counter, marker, distance)
    else:
      counter, marker = solution(counter, marker, distance)

    if Step_camera:
      cv2.imshow('frame', frame)
      cv2.imshow('edge', edged)
      k = cv2.waitKey(5) & 0xFF
      if k == 27:
        break
    # end if

  cap.release()
  cv2.destroyAllWindows()

def character(counter, marker, distance):
  print('Starting recognition thread')
  guesses = [0] * 35  # create a list of 35 lists

  for i in range(1, counter + 1):
    try:
      allContoursWithData = []  # declare empty lists
      validContoursWithData = []  # we will fill these shortly

      # set heights and width to be able to read the image when comparing to flatten images
      h = 30
      w = 30

      img = cv2.imread("colour%d.png" % i)

      height, width, numchannels = img.shape

      roi = img[int((height / 2) - (height / 2) * 0.85):int((height / 2) + (height / 2) * 0.85),
            int((width / 2) - (width / 2) * 0.85):int((width / 2) + (width / 2) * 0.85)]

      resize = cv2.resize(roi, (100, 100))

      # Convert the image to grayscale and turn to outline of the letter
      gray = cv2.cvtColor(resize, cv2.COLOR_BGR2GRAY)

      newheight, newwidth = gray.shape

      # imgMaxContrastGrayscale = maximizeContrast(gray)

      ###########

      gauss = cv2.GaussianBlur(gray, (5, 5), 0)
      # thresh = cv2.adaptiveThreshold(gauss, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 0)
      kernel = np.ones((4, 4), np.uint8)
      # mask = cv2.inRange(gauss, 170, 255)
      edged = cv2.Canny(gauss, 10, 30)  # the lower the value the more detailed it would be
      dilate = cv2.dilate(edged, kernel, iterations=1)
      kernel = np.ones((3, 3), np.uint8)
      open = cv2.morphologyEx(dilate, cv2.MORPH_OPEN, kernel, iterations=1)
      close = cv2.morphologyEx(open, cv2.MORPH_CLOSE, kernel, iterations=3)
      dilation = cv2.dilate(close, kernel, iterations=4)
      kernel = np.ones((4, 4), np.uint8)
      erode = cv2.erode(dilation, kernel, iterations=4)
      # open = cv2.morphologyEx(erode, cv2.MORPH_OPEN, kernel, iterations=1)
      # Removes the noises on the grayscale image
      denoised = cv2.fastNlMeansDenoising(erode, None, 10, 7, 21)

      mask = cv2.inRange(gray, 100, 255)  # works in lab, 100 at home,
      # cv2.waitKey(0)

      _, otsu = cv2.threshold(gauss, 0 ,255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

      # imgBlurred = cv2.GaussianBlur(gray, (5, 5), 0)                    # blur

      if Step_letter:
        cv2.imshow("mask", mask)
        cv2.imshow("gg",denoised)
        cv2.imshow("img",img)
        cv2.imshow("gray",gray)
        cv2.imshow("ed", edged)
        cv2.imshow("dil", dilate)
        cv2.imshow("otsu", otsu)
        cv2.waitKey(0)


      # Fill in the letter to detect the letter easily
      # kernel = np.ones((4, 4), np.uint8)
      # closing = cv2.morphologyEx(denoised, cv2.MORPH_CLOSE, kernel)
      # dilation = cv2.dilate(closing, kernel, iterations=1)

      knn = cv2.ml.KNearest_create()  # initalise the knn
      # joins the train data with the train_labels
      knn.train(npaFlattenedImages, cv2.ml.ROW_SAMPLE, npaClassifications)

                                                          # filter image from grayscale to black and white
      imgThresh = cv2.adaptiveThreshold(gauss,                           # input image
                                        255,                                  # make pixels that pass the threshold full white
                                        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,       # use gaussian rather than mean, seems to give better results
                                        cv2.THRESH_BINARY,                # invert so foreground will be white, background will be black
                                        11,                                   # size of a pixel neighborhood used to calculate threshold value
                                        0)                                    # constant subtracted from the mean or weighted mean

      newkernal = np.ones((3, 3), np.uint8)
      opening = cv2.morphologyEx(imgThresh, cv2.MORPH_OPEN, newkernal, iterations=1)
      eroding = cv2.erode(opening, kernel, iterations=1)
      dilating = cv2.dilate(eroding, kernel, iterations=1)

      imgThreshCopy = otsu.copy()        # make a copy of the thresh image, this in necessary b/c findContours modifies the image

      cv2.imwrite(
        'C:/Users/kevin/Desktop/2018-2019/method A/otsu/{0}_{1}contour.png'.format(marker, i), imgThreshCopy)

      (npaContours,_) = cv2.findContours(imgThreshCopy,             # input image, make sure to use a copy since the function will modify this image in the course of finding contours
                                                   cv2.RETR_LIST,         # retrieve the outermost contours only
                                                   cv2.CHAIN_APPROX_SIMPLE)   # compress horizontal, vertical, and diagonal segments and leave only their end points

      if Step_letter:
        cv2.imshow("npaContours", imgThreshCopy)
        cv2.imshow("planb", imgThresh)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

      for npaContour in npaContours:                             # for each contour
          contourWithData = ContourWithData()                                             # instantiate a contour with data object
          contourWithData.npaContour = npaContour                                         # assign contour to contour with data
          contourWithData.boundingRect = cv2.boundingRect(contourWithData.npaContour)     # get the bounding rect
          contourWithData.calculateRectTopLeftPointAndWidthAndHeight()                    # get bounding rect info
          contourWithData.fltArea = cv2.contourArea(contourWithData.npaContour)           # calculate the contour area
          allContoursWithData.append(contourWithData)                                     # add contour with data object to list of all contours with data
      # end for

      for contourWithData in allContoursWithData:                 # for all contours
          if contourWithData.checkIfContourIsValid(newheight, newwidth):             # check if valid
              validContoursWithData.append(contourWithData)       # if so, append to valid contour list
          # end if
      # end for

      validContoursWithData.sort(key=operator.attrgetter("intRectX"))  # sort contours from left to right
      validContoursWithData = removeInnerOverlappingChars(validContoursWithData) # removes overlapping letters

      for contourWithData in validContoursWithData:            # for each contour
          new = cv2.cvtColor(cv2.rectangle(roi,  # draw rectangle on original testing image
                                         (contourWithData.intRectX, contourWithData.intRectY),  # upper left corner
                                         (contourWithData.intRectX + contourWithData.intRectWidth,
                                          contourWithData.intRectY + contourWithData.intRectHeight),
                                         # lower right corner
                                         (0, 255, 0),  # green
                                         2), cv2.COLOR_BGR2GRAY)  # thickness

          imgROI = otsu[contourWithData.intRectY + 1: contourWithData.intRectY + contourWithData.intRectHeight - 1,     # crop char out of threshold image
                             contourWithData.intRectX + 1: contourWithData.intRectX + contourWithData.intRectWidth - 1]

          imgROIResized = cv2.resize(imgROI, (w, h))             # resize image, this will be more consistent for recognition and storage

          cv2.imwrite(
            'C:/Users/kevin/Desktop/2018-2019/method A/otsu/{0}_{1}chosen.png'.format(marker, i), imgROIResized)

          # for i in range(0, 360, 90):
          #   angle = i
          #   rotate = cv2.getRotationMatrix2D((w / 2, h / 2), angle, 1.0)
          #   imgROIResized = cv2.warpAffine(imgROIResized, rotate, (w, h))

          npaROIResized = imgROIResized.reshape((1, w * h))      # flatten image into 1d numpy array

          npaROIResized = np.float32(npaROIResized)       # convert from 1d numpy array of ints to 1d numpy array of floats

          if Step_letter:
            cv2.imshow("resize", imgROIResized)
            cv2.imshow("imgTestingNumbers", img)  # show input image with green boxes drawn around found digits
            cv2.waitKey(0)
          # end if

          # looks for the 3 nearest neighbours comparing to the flatten images (k = neighbours)
          retval, npaResults, neigh_resp, dists = knn.findNearest(npaROIResized, k=1)

          # current guess
          gg = int(npaResults[0][0])
          if Step_letter:
            print(gg)
          # Tranform guess in ASCII format into range 0-35
          if 49 <= gg <= 57:
            guesses[gg - 49] += 1
          elif 65 <= gg <= 90:
            guesses[gg - 56] += 1
    except:
      continue

  # find modal character guess
  # Initialise mode and prev variables for first loop through
  if Step_letter:
    print(guesses)
  mode = 0
  prev = guesses[0]
  for j in range(35):
    new = guesses[j]
    if new > prev:
      prev = guesses[j]
      mode = j
  # Transform back into ASCII
  if 0 <= mode <= 8:
    mode = mode + 49
  elif 9 <= mode <= 34:
    mode = mode + 56

  return chr(mode)

def colour(counter, marker, distance):

  results = [] # empty list
  for k in range(1, counter + 1):
      num_storage = [] # store tuple colour's value in RGB
      name_storage = [] # stores the colour's name
      img = cv2.imread("colour%d.png" % k)

      height, width, numchannels = img.shape

      roi = img[int((height / 2) - (height / 2) * 0.85):int((height / 2) + (height / 2) * 0.85),
            int((width / 2) - (width / 2) * 0.85):int((width / 2) + (width / 2) * 0.85)]

      # img = cv2.imread("colour%d.png" % k, cv2.COLOR_BGR2RGB)
      Gauss = cv2.GaussianBlur(roi, (5, 5), 0)
      # blur = cv2.medianBlur(Gauss, 7)
      # fliter = cv2.bilateralFilter(blur, 15, 75, 75)
      kernel = np.ones((10, 10), np.uint8)
      erode = cv2.erode(Gauss, kernel, iterations=10)
      dilation = cv2.dilate(erode, kernel, iterations=20)
      denoised = cv2.fastNlMeansDenoisingColored(dilation, None, 10, 10, 7, 21)

      h = 8
      w = 8
      # h, w = img[:2]

      current_mode = 0

      # # defined boundaries HSV
      # boundaries = [("black", [0, 0, 0], [179, 255, 50]), ("white", [0, 0, 185], [179, 30, 255]),
      #                 ("orange", [10, 30, 50], [25, 255, 255]), ("yellow", [25, 30, 50], [35, 255, 255]),
      #                 ("green", [35, 30, 50], [85, 255, 255]), ("blue", [85, 30, 50], [130, 255, 255]),
      #                 ("purple", [130, 30, 50], [145, 255, 255]), ("pink", [145, 30, 50], [165, 255, 255]),
      #                 ("red", [165, 30, 50], [179, 255, 255]), ("red", [0, 30, 50], [10, 255, 255]),
      #                 ("grey", [0, 0, 50], [179, 30, 185])]

      # # defined boundaries RGB
      # boundaries = [("black", [0, 0, 0]), ("white", [255, 255, 255]),
      #               ("orange", [255, 165, 0]), ("yellow", [255, 255, 0]),
      #               ("green", [0, 128, 0]), ("blue", [0, 0, 255]),
      #               ("purple", [128, 0, 128]), ("pink", [255, 192, 203]),
      #               ("red", [255, 0, 0]), ("grey", [128, 128, 128]),
      #               ("aqua", [0, 255, 255]), ("fuchsia", [255, 0, 255]),
      #               ("silver", [192, 192, 192]), ("maroon", [128, 0, 0]),
      #               ("olive", [128, 128, 0]), ("lime", [0, 255, 0]),
      #               ("teal", [0, 128, 128]), ("navy", [0, 0, 128]),]

      # # defined boundaries RGB
      # boundaries = [("black", [0, 0, 0]), ("white", [255, 255, 255]),
      #               ("yellow", [255, 255, 0]), ("purple", [128, 0, 128]),
      #               ("green", [0, 128, 0]), ("blue", [0, 0, 255]),
      #               ("red", [255, 0, 0]), ("grey", [128, 128, 128]),
      #               ("blue", [0, 255, 255]), ("pink", [255, 0, 255]),
      #               ("grey", [192, 192, 192]), ("red", [128, 0, 0]),
      #               ("yellow", [128, 128, 0]), ("green", [0, 255, 0]),
      #               ("blue", [0, 128, 128]), ("blue", [0, 0, 128])]

      # # defined boundaries HSV
      # boundaries = [("black", [0, 0, 0]), ("white", [0, 0, 255]),
      #               ("yellow", [30, 255, 255]), ("purple", [150, 255, 127]),
      #               ("green", [60, 255, 127]), ("blue", [120, 255, 255]),
      #               ("red", [0, 255, 255]), ("grey", [0, 0, 127]),
      #               ("blue", [90, 255, 255]), ("pink", [150, 255, 255]),
      #               ("grey", [0, 0, 191]), ("red", [0, 255, 127]),
      #               ("yellow", [30, 255, 127]), ("green", [60, 255, 255]),
      #               ("blue", [90, 255, 127]), ("blue", [120, 255, 127])]

      resizeBGR = cv2.resize(denoised, (w, h)) # reduces the size of the image so the process would run fast

      cv2.imwrite(
        'C:/Users/kevin/Desktop/2018-2019/method A/color/{0}_{1}.png'.format(marker, k), denoised)

      # print(resizeBGR[1,1])

      # resizeHSV = cv2.cvtColor(resizeBGR, cv2.COLOR_BGR2HSV)
      resizeRGB = cv2.cvtColor(resizeBGR, cv2.COLOR_BGR2RGB)

      # print(resizeHSV[1,1])

      if Step_color:
        for x in range(0, w):
          for y in range(0, h):
            num_storage.append(resizeHSV[x, y])

        print(num_storage)

        cv2.imshow("fliter", dilation)
        cv2.imshow("denos", denoised)
        cv2.imshow("resize", resizeBGR)
        cv2.waitKey(0)
      # end if

      # for i in range(0, h):
      #   for j in range(0, w):
      #     RGB = resizeHSV[i, j]
      #     differences = []
      #     for (name, value) in boundaries:
      #       for component1, component2 in zip(RGB, value):
      #         difference = sum([abs(component1 - component2)])
      #         differences.append([difference, name])
      #     differences.sort()
      #     name_storage.append(differences[0][1])
      #
      # majority = Counter(name_storage)
      # results.append(majority.most_common(1)[0][0])

      for i in range(0, h):
        for j in range(0, w):
          # RGB = []
          RGB = resizeRGB[i, j]
          # num_storage.append(RGB)
          # Finds the nearest colour name within the webcolors dataset
          try:
            colorname = webcolors.rgb_to_name(RGB)
            name_storage.append(colorname)

          except ValueError:
            min_colours = {}
            for key, name in webcolors.css3_hex_to_names.items():
              r_c, g_c, b_c = webcolors.hex_to_rgb(key)
              rd = (r_c - RGB[0]) ** 2
              gd = (g_c - RGB[1]) ** 2
              bd = (b_c - RGB[2]) ** 2
              min_colours[(rd + gd + bd)] = name
            name_storage.append(min_colours[min(min_colours.keys())])

      majority = Counter(name_storage)
      results.append(majority.most_common(1)[0][0])

      # # comparing each pixel of the picture and append the colour name in to a list (BGR to RGB to get the name)
      # for (color, lower, upper) in boundaries:
      #   lower = np.array(lower, dtype="uint8")
      #   upper = np.array(upper, dtype="uint8")
      #
      #   mask = cv2.inRange(resizeHSV, lower, upper)
      #
      #   ratio = np.round((cv2.countNonZero(mask) / (resizeHSV.size / 3))*100, 2)
      #   if ratio > current_mode:
      #     current_mode = ratio
      #     name_storage.append(color)
      #   else:
      #     pass
      # results.append(name_storage[-1])

  mode = Counter(results)

  if mode == Counter():
    colourname = "None"
  else:
    colourname = mode.most_common(1)[0][0]

  return colourname

def main():
  if GPS:
    print('Connecting to drone...')

    # Connect to vehicle and print some info
    vehicle = connect('192.168.0.156:14550', wait_ready=True, baud=921600)

    print('Connected to drone')
    print('Autopilot Firmware version: %s' % vehicle.version)
    print('Global Location: %s' % vehicle.location.global_relative_frame)
  detection()

def maximizeContrast(imgGrayscale):
    height, width = imgGrayscale.shape

    imgTopHat = np.zeros((height, width, 1), np.uint8)
    imgBlackHat = np.zeros((height, width, 1), np.uint8)

    structuringElement = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))

    imgTopHat = cv2.morphologyEx(imgGrayscale, cv2.MORPH_TOPHAT, structuringElement)
    imgBlackHat = cv2.morphologyEx(imgGrayscale, cv2.MORPH_BLACKHAT, structuringElement)

    imgGrayscalePlusTopHat = cv2.add(imgGrayscale, imgTopHat)
    imgGrayscalePlusTopHatMinusBlackHat = cv2.subtract(imgGrayscalePlusTopHat, imgBlackHat)

    return imgGrayscalePlusTopHatMinusBlackHat

def removeInnerOverlappingChars(listOfMatchingChars):
  # if we have two chars overlapping or to close to each other to possibly be separate chars, remove the inner (smaller) char,
  # this is to prevent including the same char twice if two contours are found for the same char,
  # for example for the letter 'O' both the inner ring and the outer ring may be found as contours, but we should only include the char once
  listOfMatchingCharsWithInnerCharRemoved = list(listOfMatchingChars)  # this will be the return value

  for currentChar in listOfMatchingChars:
    for otherChar in listOfMatchingChars:
      if currentChar != otherChar:  # if current char and other char are not the same char . . .
        # if current char and other char have center points at almost the same location . . .
        if distanceBetweenChars(currentChar, otherChar) < (currentChar.fltDiagonalSize * 0.3):
          # if we get in here we have found overlapping chars
          # next we identify which char is smaller, then if that char was not already removed on a previous pass, remove it
          if currentChar.fltArea < otherChar.fltArea:  # if current char is smaller than other char
            if currentChar in listOfMatchingCharsWithInnerCharRemoved:  # if current char was not already removed on a previous pass . . .
              listOfMatchingCharsWithInnerCharRemoved.remove(currentChar)  # then remove current char
            # end if
          else:  # else if other char is smaller than current char
            if otherChar in listOfMatchingCharsWithInnerCharRemoved:  # if other char was not already removed on a previous pass . . .
              listOfMatchingCharsWithInnerCharRemoved.remove(otherChar)  # then remove other char
            # end if
          # end if
        # end if
      # end if
    # end for
  # end for

  return listOfMatchingCharsWithInnerCharRemoved

def distanceBetweenChars(firstChar, secondChar):
  # use Pythagorean theorem to calculate distance between two chars
  intX = abs(firstChar.intCentreX - secondChar.intCentreX)
  intY = abs(firstChar.intCentreY - secondChar.intCentreY)

  return math.sqrt((intX ** 2) + (intY ** 2))

if __name__ == "__main__":
    # GPS
    GPS = False
    air = 1

    # Steps
    Step_camera = False
    Step_detection = False
    Step_letter = False
    Step_color = False
    Static_Test = False

    # Variable
    MIN_CONTOUR_AREA = 100
    camera_lens_angle = 62.2/2

    # Load training and classification data
    npaClassifications = np.loadtxt("classwithi.txt", np.float32)
    npaFlattenedImages = np.loadtxt("flatwithi.txt", np.float32)

    # reshape classifications array to 1D for k-nn
    npaClassifications = npaClassifications.reshape((npaClassifications.size, 1))

    main()
