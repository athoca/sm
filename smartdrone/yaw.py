import cv2
import numpy as np
import math
from smartdrone.landingpad import compute_target_from_frame

def estimate_H_area(H, ratio = 1.5):
    # H in meters
    # (1199.5*(6.14**2) + 1822*(4.92**2) + 4240*(3.12**2) + 6715*(2.39**2) + 7163*(2.4**2) + 6690*(2.52**2) + 6423*(2.48**2)) / 7
    average = 41743/(H**2)
    return average*ratio, average/ratio

def estimate_cirle_area(H, ratio = 1.7): # because detection is smaller than full landing pad
    # H in meters
    # (11162*6.14**2 + 16752*4.92**2 + 70033*2.4**2)/3
    average = 409900/(H**2)
    return average*ratio, average/ratio

def get_outer_white_contours(contours, area_max=8000, area_min = 4000):
    for cnt in contours:
        area = cv2.contourArea(cnt) 
#         print(area)
        if area < area_max and area > area_min:
            return cnt
    return None

def preprocess_frame_for_yaw_detection(bgr_img, H):
    # https://docs.opencv.org/3.1.0/d9/d8b/tutorial_py_contours_hierarchy.html
    # For calculate yaw direction
    hsv = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2HSV)
    ret, thresh = cv2.threshold(hsv[:,:,1], 50, 255, 0)
#     plt.imshow(thresh,cmap = 'gray')
#     plt.show()
    
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    area_max, area_min = estimate_H_area(H)
    cnt = get_outer_white_contours(contours, area_max, area_min)
    # fill inner of cnt with white
#     cv2.fillPoly(bgr_img, [cnt], [255,255,255])
    
    mask = np.zeros(thresh.shape, dtype=np.uint8)
    cv2.fillPoly(mask, [cnt], [255])    
#     plt.imshow(mask,cmap = 'gray')
#     plt.show()
    
    masked_img = np.zeros(thresh.shape, dtype=np.uint8)
#     masked_img[np.where(mask == 255)] = hsv[:,:,1][np.where(mask == 255)]
    masked_img[np.where(mask == 255)] = thresh[np.where(mask == 255)]
#     plt.imshow(masked_img,cmap = 'gray')
#     plt.show()
    
#     # https://docs.opencv.org/master/dc/d0d/tutorial_py_features_harris.html
#     dst = cv2.cornerHarris(masked_img,3,7,0.05)
#     bgr_img[dst>0.05*dst.max()]=[255,0,0]
    
    # https://docs.opencv.org/3.4/d6/d10/tutorial_py_houghlines.html
    # https://docs.opencv.org/3.4/da/d22/tutorial_py_canny.html
    edges = cv2.Canny(masked_img,10,200,apertureSize = 3)    
#     plt.imshow(edges,cmap = 'gray')
#     plt.show()

#     lines = cv2.HoughLines(edges,0.5,np.pi/180,40) # 0.5 for H~3m, 1.2 for H~6m
#     lines = cv2.HoughLines(edges,1.2,np.pi/180,40)
    lines = cv2.HoughLines(edges,1.4,np.pi/180,40)

    
    # For computing beacon on top or bottom
    area_max, area_min = estimate_cirle_area(H)
    cnt = get_outer_white_contours(contours, area_max, area_min)
    
    mask = np.zeros(thresh.shape, dtype=np.uint8)
    cv2.fillPoly(mask, [cnt], [255])
    
    masked_img = np.zeros(thresh.shape, dtype=np.uint8)
    masked_img[np.where(mask == 255)] = hsv[:,:,2][np.where(mask == 255)]
#     plt.imshow(masked_img,cmap = 'gray')
#     plt.show()    
    return masked_img, lines[:21] # only based 21 first = 21 best as max

def is_0_point(nums):
    for num in nums:
        if num < 4:
            for mum in nums:
                if mum > 176:
                    return True
            return False
    return False

def shift_90(nums):
    return [(num+90) % 180 for num in nums]

def median(nums):
    nums = sorted(nums)
    middle1 = (len(nums) - 1) // 2
    middle2 = len(nums) // 2
    return (nums[middle1] + nums[middle2]) / 2

def compute_yaw_direction(arr, e=4, ratio_min = 0.5):
    arr = arr * 180/math.pi # convert to degree
    arr = [int(round(i)) for i in arr]
#     print(arr)
    _min = int(ratio_min* len(arr))
    for i in range(len(arr)):
        v = arr[i]
        count = 0
        vs = []
        for u in arr:
            if (abs(v-u) < e) or (abs(v-u) > 180 - e):
                count += 1
                vs.append(u)
        if len(vs) > _min:
            if is_0_point(vs):
                new_vs = shift_90(vs)
                return (median(new_vs) - 90) % 180
            return median(vs)
    return None

def rotate_image(image, angle):
    # In CCW
    image_center = tuple(np.array(image.shape[1::-1]) / 2)
    rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
    result = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
    return result

def compute_yaw_value(masked_img, angle):
    rotated = rotate_image(masked_img, angle) # In CCW
    ret, rotated = cv2.threshold(rotated, 150, 255, 0)
    # plt.imshow(rotated, cmap = 'gray')
    # plt.show()
    size = rotated.shape[0]
    c = int(size/2)

    top_sum = np.sum(rotated[c-40:c, c-15:c+15])
    bottom_sum = np.sum(rotated[c:c+40, c-15:c+15])
    
    if top_sum > bottom_sum: # beacon in bottom part
        return angle # drone yaw in CW 
    else:
        return angle + 180 # drone yaw in CW

def detect_yaw(bgr_img, H):
    masked_img, lines = preprocess_frame_for_yaw_detection(bgr_img, H)
    angle = compute_yaw_direction(lines[:,0,1])
    if angle is not None:
        yaw_angle = compute_yaw_value(masked_img, angle)
        return yaw_angle
    else:
        return 0

def compute_yaw_frome_frame(RGB_img, H, is_gimbal_rotated):
    heading = 0
    _, _, l, t, w, h = compute_target_from_frame(RGB_img, H, heading, is_gimbal_rotated, ratio=2)
    if l is not None:
        l = max(0,l)
        t = max(0,t)
        b = min(t+h, RGB_img.shape[0])
        r = min(l+w, RGB_img.shape[1])
        BGR_img = cv2.cvtColor(RGB_img, cv2.COLOR_BGR2RGB)
        landing_pad_img = BGR_img[t:b, l:r]
        try:
            yaw_angle = detect_yaw(landing_pad_img, H)
        except Exception as e:
            print("ERROR when compute yaw angle")
            yaw_angle = 0
        return yaw_angle, landing_pad_img
    else:
        return 0, None