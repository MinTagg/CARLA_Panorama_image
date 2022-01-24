import cv2
import numpy as np
import os, glob
#import matplotlib.pyplot as plt

def cylindricalWarp(img, K, lr):
    """This function returns the cylindrical warp for a given image and intrinsics matrix K"""
    #print('Hi')
    h_,w_ = img.shape[:2]
    # pixel coordinates
    y_i, x_i = np.indices((h_,w_))
    X = np.stack([x_i,y_i,np.ones_like(x_i)],axis=-1).reshape(h_*w_,3) # to homog
    Kinv = np.linalg.inv(K) 
    X = Kinv.dot(X.T).T # normalized coords
    # calculate cylindrical coords (sin\theta, h, cos\theta)
    A = np.stack([np.sin(X[:,0]),X[:,1],np.cos(X[:,0])],axis=-1).reshape(w_*h_,3)
    B = K.dot(A.T).T # project back to image-pixels plane
    # back from homog coords
    B = B[:,:-1] / B[:,[-1]]
    # make sure warp coords only within image bounds
    B[(B[:,0] < 0) | (B[:,0] >= w_) | (B[:,1] < 0) | (B[:,1] >= h_)] = -1
    B = B.reshape(h_,w_,-1)
    
    #print("ordinary :: ", h_, w_)
    #print("B shape :: ", B.shape)

    img_rgba = cv2.cvtColor(img,cv2.COLOR_BGR2BGRA) # for transparent borders...
    # warp the image according to cylindrical coords

    result, lr_out = cut_side(cv2.remap(img_rgba, B[:,:,0].astype(np.float32), B[:,:,1].astype(np.float32), cv2.INTER_AREA, borderMode=cv2.BORDER_TRANSPARENT),B, lr)
    #print('######## Result Value ########')
    #print("result shape :: ",result.shape)
    #print("result type :: ", type(result))
    #print("lr length :: ",len(lr_out))
    #print("lr type :: ", type(lr_out))
    if lr == None:
        # if point is not defined, return left and right point
        return result, lr_out
    else:
        return result

def cut_side(img,b, lr):

    if lr == None:
        b0 = b[:,:,0]
        b1 = b[:,:,1]

        _,b = b0.shape

        for i in range(b):
            if np.array_equal(b0[:,i], b1[:,i]) == False:
                #print(i)
                #left = i
                break
  
        for j in reversed(range(b)):
            #print(j)
            if np.array_equal(b0[:,j], b1[:,j]) == False:
                #print(j)
                #right = j
                break 
  
        return img[:,i:j], [i, j]
    else:
        return img[:, lr[0]:lr[1]], [lr[0], lr[1]]


def cut_up(img, point):
    length,_,_ = img.shape
    if point == None:
        # if point is not defined, return image and point
        for i in range(length):
            if np.array_equal(img[0,0,:], img[i,0,:]) == False:
                up = i
                break

        for i in reversed(range(length)):
            if np.array_equal(img[0,0,:], img[i,0,:]) == False:
                down = i
                break

        #print("up :: ", up)
        #print('down :: ', down)
        return img[up:down,:], [up,down]
    else:
        return img[point[0]:point[1],:]

def make_image(imgs, point = None, f = 570):
    # make_image get 4 images by list
    # f value can be changed
    # point is consist of 4 values[up, down, left, right]
    #print('##### make_image check #######')
    #print(imgs[0],'\n')
    h,w = imgs[0].shape[:2]
    K = np.array([[f,0,w/2],[0,f,h/2],[0,0,1]]) # mock intrinsics

    #print(imgs[0].shape)

    if point == None:
        # if point is not defined, plot image and find the point
        while True:
            #print(imgs[0].shape)
            #print(type(imgs[0]))
            #print(imgs[0])
            temp0, left_right = cylindricalWarp(imgs[0],K, point)
            #plt.figure(figsize=(20, 20))
            #plt.imshow(cv2.cvtColor(temp0, cv2.COLOR_BGR2RGB))
            cv2.imshow("",temp0)
            cv2.waitKey(0)
            sat = input("If first Image is fine input yes :: ")
            if sat == 'yes':
                break
        temp1 = cylindricalWarp(imgs[1],K, left_right)
        temp2 = cylindricalWarp(imgs[2],K, left_right)
        temp3 = cylindricalWarp(imgs[3],K, left_right)
    else :
        # point is defined
        temp0 = cylindricalWarp(imgs[0],K, [point[2], point[3]])
        temp1 = cylindricalWarp(imgs[1],K, [point[2], point[3]])
        temp2 = cylindricalWarp(imgs[2],K, [point[2], point[3]])
        temp3 = cylindricalWarp(imgs[3],K, [point[2], point[3]])
    
    together = np.hstack((temp0, temp1, temp2, temp3))

    if point == None:
        result , up_down = cut_up(together, point)
        return result, [up_down[0], up_down[1], left_right[0], left_right[1]]
    else:
        result = cut_up(together, [point[0], point[1]])
        return result



