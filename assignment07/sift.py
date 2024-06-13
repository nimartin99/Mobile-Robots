import numpy as np
import cv2 as cv

images = ['blox.jpg', 'morgenstelle.png', 'morgenstelle_blurry.png']
for image in images:
    img = cv.imread(image)
    gray= cv.cvtColor(img,cv.COLOR_BGR2GRAY)
    sift = cv.SIFT_create()
    kp = sift.detect(gray,None)
    img=cv.drawKeypoints(gray,kp,img)
    cv.imwrite('sift_keypoints_ ' + image + '.jpg',img)

    print("Number of Keypoints in image " +  image +": ", len(kp))