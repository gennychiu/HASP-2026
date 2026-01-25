#img = numpy.asarray(img_array)
#img = img.reshape(960, 1280)
#cv2.imshow('Image From Guide Camera', img)
#cv2.waitKey()


numpy.frombuffer(img_array)

cv2.imdecode(numpy.frombuffer(img_array), 0)

img1 = numpy.asarray(img_array)
img1 = img1.reshape(960, 1280)
cv2.imshow('img', img1)
cv2.waitKey()


#cvImg = numpy.zeros((960, 1280, img_array), dtype = "uint8")


#numpy.mat(img_array, numpy.byte, 960, 1280)


vis = numpy.zeros((960, 1280), numpy.byte)
vis = img_array
h,w = vis.shape
vis2 = cv2.UMat(h, w, cv2.CV_32FC3)






img_gray = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
# apply binary thresholding
ret, thresh = cv2.threshold(img_gray, 100, 255, cv2.THRESH_BINARY)
# visualize the binary image
cv2.imshow('Binary image', thresh)
cv2.waitKey(0)
# cv2.imwrite('image_thres1.jpg', thresh)


#img_array = AsiModule.get_guide_image()
#from PIL import Image
#img_from_guide_camera = Image.frombuffer("L", (1280, 960), img_array)
#opencv_image = cv2.cvtColor(numpy.array(img_from_guide_camera), cv2.COLOR_RGB2BGR)


#cv2.imshow('Image From Guide Camera', guider_image)
#cv2.waitKey()


#img_from_file = cv2.imread(".\\SunPic01.jpg")
#img_resized = cv2.resize(img_from_file, (0, 0), fx = 0.45, fy = 0.45)
#cv2.imshow("Image From File", img_resized)
#cv2.waitKey()

#img_resized = guider_image

#img_gray = cv2.cvtColor(img_resized, cv2.COLOR_BGR2GRAY)
#cv2.imshow("Image From File in Gray", img_gray)
#cv2.waitKey()

#ret, img_thresh = cv2.threshold(img_gray, 100, 255, cv2.THRESH_TRIANGLE)
#cv2.imshow("Threshold Image", img_thresh)
#cv2.waitKey()

#contours, hierarchy = cv2.findContours(img_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#cv2.drawContours(img_resized, contours, -1, (0, 255, 0), 1)

# loop through all contours, calculate measurements and store values in appropriate list
#areas = []
#centers = []
#moments_list = []
#largestarea = 0.0
#for c in contours:
#    area = cv2.contourArea(c)
#    areas.append(cv2.contourArea(c))
#    if (area > largestarea):
#        largestarea = area
#        M_1= cv2.moments(c)
#        moments_list.append(M_1)
#        if M_1["m00"] == 0: M_1["m00", "m01"] = 1
#        x = int(M_1["m10"] / M_1["m00"])
#        y = int(M_1["m01"] / M_1["m00"])
#        centers.append((x, y))

#cv2.circle(img_resized,(x, y), 2, (255,0,0), -1)

#cv2.imshow('Image with Contours', img_resized)
#cv2.waitKey(0)

#print('x error = ', x_err)
#
#print('y error = ', y_err)
