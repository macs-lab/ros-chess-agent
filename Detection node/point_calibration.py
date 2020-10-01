import cv2
import numpy as np
import math
from collections import Counter 
from itertools import combinations 
from math import sqrt
import os

def min_max(num):
    if num%2 != 0:
        minm = num//2 - 300
        maxm = num//2 + 300
        return int(minm), int(maxm)
    else:
        minm = num/2 - 299
        maxm = num/2 + 300
        return int(minm),int(maxm)
        
def crop_image(img):
    """ crop the given image
    return the new image and also the coordinate of the upper left corner pixel the new image in the previous image """
    width = img.shape[1]
    height = img.shape[0]
    minw,maxw = min_max(width)
    minh,maxh = min_max(height)
    cropped_img=img[minh:maxh,minw:maxw,:]
    ori = np.array([minh,minw])
    return cropped_img, ori
    
def image_processing(img):
    """ process image for line detection
    return a black backward and white lines picture """
    img = cv2.GaussianBlur(img,(11,11),0)
    img_th = cv2.adaptiveThreshold(img,255,cv2.ADAPTIVE_THRESH_MEAN_C,cv2.THRESH_BINARY,5,2)
    img_blur = cv2.medianBlur(img_th,5)
    img_invert = cv2.bitwise_not(img_blur)
    kernel_1 = np.array([[0,1,0],[1,1,1],[0,1,0]], np.uint8) 
    kernel_2 = np.ones((5,5),np.uint8)
    img_d = cv2.dilate(img_invert, kernel_1, iterations=1) 
    img_m = cv2.morphologyEx(img_d, cv2.MORPH_OPEN, kernel_2)
    img_e = cv2.erode(img_m,kernel_2,iterations = 1)
    return img_e

def line_detection(img_e):
    """ detect lines 
    return list of start and end pixels' coordinate of lines """
    canny = cv2.Canny(img_e, 50, 150, apertureSize=3)
    lines = cv2.HoughLinesP(canny, 1, np.pi / 180, 120)
    return lines

def rho_theta(x1,y1,x2,y2):
    """ use start and end pixels' coordinate to calculate a line's theta and rho
    return theta and rho """
    theta = np.arctan2((x2-x1),abs(y1-y2))
    rho = x1*np.cos(theta)+y1*np.sin(theta)
    return rho,theta

def most_frequent(List): 
    """ return the most commone element in the list """
    occurence_count = Counter(List) 
    return occurence_count.most_common(1)[0][0] 

def contour_corner(lines):
    """ find the four corners of the chessboard
    determine the most feasible edges of the chessboard
    calculate the intersection of intersection of four edges
    return a list of corners coordinate """
    edge = {}
    for line in lines:
        x1,y1,x2,y2 = line[0] 
        rho,theta = rho_theta(x1,y1,x2,y2)
        theta = round(theta,3)
        key_t = math.floor(theta * 10 ** 1) / 10 ** 1
        try:
            edge[key_t][0].append(theta)
            edge[(key_t)][1].append(rho)
        except:
            edge[key_t] = [[theta],[rho]]

    border = []
    for slope in edge:
        theta = most_frequent(edge[slope][0])
        rho_max = max(edge[slope][1])
        rho_min = min(edge[slope][1])
        border.append((theta,rho_max))
        border.append((theta,rho_min))
    comb = combinations(border, 2) 
    intersection = [i for i in list(comb) if i[0][0] != i[1][0]]

    cnt = np.empty((0,2),int)
    for p in intersection:
        theta1,rho1 = p[0][0],p[0][1]
        theta2,rho2 = p[1][0],p[1][1]
        cos1,sin1 = np.cos(theta1),np.sin(theta1)
        cos2,sin2 = np.cos(theta2),np.sin(theta2)
        y = (rho1*cos2-rho2*cos1)/(sin1*cos2-sin2*cos1)
        x = rho1/cos1 - y*sin1/cos1
        cnt = np.append(cnt,[[int(x),int(y)]],axis = 0)
    cnt = cnt[np.argsort(cnt[:, 0])]
    return cnt

def crop_area(img,cnt):
    """ crop the image again so that the entire image is only the chessboard
    return a image and coordinate of the upper left corner of the new image in the previous image """
    rect = cv2.minAreaRect(cnt)
    
    if abs(rect[2])%90 == 0:
        rect = (rect[0],rect[1],0)
    else:
        rotation  = rect[2]
    
    box = cv2.boxPoints(rect)
    box = np.int0(box)
    col = [i[0] for i in box]
    row = [i[1] for i in box]
    cropped = img[min(row):max(row),min(col):max(col),:]

    ori = np.array([min(row),min(col)])
    return cropped,ori

def reference_point(test,ori):
    """ determine each square's coordinate on the chessboard
    it will ask user whether the detected point is correct
    return a dictionary that has name and coordinate information of each square """
    img = test.copy()
    gray_op = cv2.cvtColor(test, cv2.COLOR_BGR2GRAY)
    corners = cv2.goodFeaturesToTrack(gray_op, 100, 0.01, 10)
    corners = np.int0(corners)
    _,column,_ = np.shape(img)
    re_row = []
    re_col = []
    for i in corners:
        x, y = i.ravel()
        if (abs(column-x)-abs(0-y))**2 < 200:
            cv2.circle(img,(x,y),3,(0,0,255),-1)
            re_row.append(y)
            re_col.append(x)
    re_row.sort()
    re_col.sort()
    cv2.imshow('PoB',img)
    k = cv2.waitKey(0)
    if k == ord('g'): 
        sk = 7
        square_dict = {}
        num = [8, 7, 6, 5, 4, 3, 2, 1]
        alphabet = ['H', 'G', 'F', 'E', 'D', 'C', 'B', 'A']
        for row in range(len(alphabet)):
            for col in range(len(num)):
                key = alphabet[row]+str(num[col])
                square = (re_col[col]+ori[0]-sk,re_col[col+1]+ori[0]+sk,
                re_row[row]+ori[1]-sk,re_col[row+1]+ori[1]+sk)
                square_dict[key] = square
        return square_dict
    else:
        return None



    

#main



cb = cv2.imread('chessboard.png',1)
cb_crop,s1 = crop_image(cb)

cb_gray = cv2.cvtColor(cb_crop, cv2.COLOR_BGR2GRAY)
cb_o = image_processing(cb_gray)
lines = line_detection(cb_o)


chessboard_corner = contour_corner(lines)

chessboard,s2 = crop_area(cb_crop,chessboard_corner)


cb_ori_in_pic = s1 + s2

ori = (cb_ori_in_pic[1],cb_ori_in_pic[0])

square_dict = reference_point(chessboard,ori)
print(square_dict)
draw_square(cb,square_dict)
