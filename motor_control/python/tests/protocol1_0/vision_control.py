#!/usr/bin/env python
from turtle import shape
import urllib.request as ur
#import rospy
#from rospy.numpy_msg import numpy_msg

#from sensor_msgs.msg import Image as SensorImage
import cv2
#from cv_bridge import CvBridge
import numpy as np
from time import sleep
#import ssl
#from imageai import Detection
'''
ctx = ssl.create_default_context()
ctx.check_hostname = False
ctx.verify_mode = ssl.CERT_NONE
'''

url = 'http://192.168.43.1:8080/shot.jpg'
#url = 'rtsp://100.66.233.138:8080'

'''modelpath = "/home/mamoon/yolo.h5"
yolo = Detection.ObjectDetection()
yolo.setModelTypeAsYOLOv3()
yolo.setModelPath(modelpath)
yolo.loadModel()
'''
'''
def publish(img):
    pub = rospy.Publisher('numpy_image', SensorImage, queue_size=10)
    rospy.init_node('cam_view',anonymous=True)
    bridge = CvBridge()
    pub.publish(bridge.cv2_to_imgmsg(img, "bgr8"))
    rospy.loginfo("image is being published")
'''
def getimage():
    while True:
        try:
            imgResp = ur.urlopen(url)
        except:
            raise
            '''
            username=""
            password=""

            #imgResp=requests.get(url, auth=HTTPBasicAuth(username, password))

            password_mgr = ur.HTTPPasswordMgrWithDefaultRealm()
            # Add the username and password.
            # If we knew the realm, we could use it instead of None.

            password_mgr.add_password(None, url, username, password)

            handler = ur.HTTPBasicAuthHandler(password_mgr)

            # create "opener" (OpenerDirector instance)
            opener = ur.build_opener(handler)

            # use the opener to fetch a URL

            opener.open(url)

            # Install the opener.
            # Now all calls to urllib.request.urlopen use our opener.
            ur.install_opener(opener)
            imgResp = ur.urlopen(url)'''
        imgNp = np.array(bytearray(imgResp.read()), dtype=np.uint8)
        img = cv2.imdecode(imgNp, -1)


        #cv2.imshow('Image',cv2.resize(img,(800,400)))
        # Convert to graycsale
        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # Blur the image for better edge detection
        img_blur = cv2.GaussianBlur(img_gray, (3,3), 0) 

        # Canny Edge Detection
        edges = cv2.Canny(image=img_blur, threshold1=100, threshold2=200) # Canny Edge Detection
        # Display Canny Edge Detection Image
        cv2.imshow('Canny Edge Detection', cv2.resize(edges,(800,400)))
        #cv2.waitKey(0)
        #vision.vision(img)
        #publish(img)
        ## press q or Esc to quit
        if (cv2.waitKey(1) & 0xFF == ord("q")) or (cv2.waitKey(1)==27):
            break
        q = cv2.waitKey(1)
        if q == ord("q"):
            break

    cv2.destroyAllWindows()

def rotate_image(img):
    # Create a zeros image
    #img = np.zeros((400,400), dtype=np.uint8)

    # Specify the text location and rotation angle
    text_location = (240,320)
    angle = 35

    # Draw the text using cv2.putText()
    font = cv2.FONT_HERSHEY_SIMPLEX
    #cv2.putText(img, 'TheAILearner', text_location, font, 1, 255, 2)

    # Rotate the image using cv2.warpAffine()
    M = cv2.getRotationMatrix2D(text_location, angle, 1)
    out = cv2.warpAffine(img, M, (img.shape[1], img.shape[0]))

    # Display the results
    #cv2.imshow('img',out)
    #cv2.waitKey(0)
    return out

def pixel_to_cartesian(pixels):
    # image pixels (235,31) to (397,31) separated by cartesian distance of 60mm
    # image pixels (317,59) to (320,288) separated by cartesian distance of 85mm
    # scale_x = 60/(397-235) = 60/162
    # lets take scale_x  = 60/143
    # similarly: scale_y = 85/206
    pixels=np.array([[pixels[0]-320],[pixels[1]-240]])
    scale_x=60/141 # linear map factor from pixel to cartesian in mm in camera frame
    scale_y=85/200
    offset_y= 120
    offset_x= 15
    rotation_cam2robot=np.array([[0,1],[-1,0]])
    #x_cartesian= pixels[0] * scale_x  + offset_x
    #y_cartesian= pixels[1] * scale_y + offset_y
    scale= np.array([[scale_x,0],[0,scale_y]])
    offset=np.array([[offset_x],[offset_y]])
    return np.dot(rotation_cam2robot,np.dot(scale,pixels)) + offset       #cartesian coordinates w.r.t robot frame in mm



def test():
    # Read the original image
    img = cv2.imread('/home/mamoon/Desktop/engineering/test_img.jpg') 
    # Display original image
    cv2.imshow('Original', cv2.resize(img,(800,400)))
    cv2.waitKey(0)

    # Convert to graycsale
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Blur the image for better edge detection
    img_blur = cv2.GaussianBlur(img_gray, (3,3), 0) 

    # Sobel Edge Detection
    sobelx = cv2.Sobel(src=img_blur, ddepth=cv2.CV_64F, dx=1, dy=0, ksize=5) # Sobel Edge Detection on the X axis
    sobely = cv2.Sobel(src=img_blur, ddepth=cv2.CV_64F, dx=0, dy=1, ksize=5) # Sobel Edge Detection on the Y axis
    sobelxy = cv2.Sobel(src=img_blur, ddepth=cv2.CV_64F, dx=1, dy=1, ksize=5) # Combined X and Y Sobel Edge Detection
    # Display Sobel Edge Detection Images
    cv2.imshow('Sobel X', cv2.resize(sobelx,(800,400)))
    cv2.waitKey(0)
    cv2.imshow('Sobel Y', cv2.resize(sobely,(800,400)))
    cv2.waitKey(0)
    cv2.imshow('Sobel X Y using Sobel() function', cv2.resize(sobelxy,(800,400)))
    cv2.waitKey(0)

    # Canny Edge Detection
    edges = cv2.Canny(image=img_blur, threshold1=100, threshold2=200) # Canny Edge Detection
    # Display Canny Edge Detection Image
    cv2.imshow('Canny Edge Detection', cv2.resize(edges,(800,400)))
    cv2.waitKey(1)

def detect_center():
    port=-1 #cam port for webcam
    video=cv2.VideoCapture(port)
    rslt,img=video.read()
    
    #480,640,3
    #print(img.shape)
    #cv2.imshow("camera_view",cv2.resize(img,(400,200)))
    cv2.imshow("rotated_cropped",img)
    cv2.waitKey(0)
    
    img=rotate_image(img)
    img=img[100:400,60:380]
    #cv2.imshow("camera_view",cv2.resize(img,(400,200)))
    cv2.imshow("rotated_cropped",img)
    cv2.waitKey(0)
    # Read the original image
    #img = cv2.imread('/home/mamoon/Downloads/test4.jpg')
    #img = cv2.imread('/home/mamoon/Desktop/engineering/test_img.jpg')
    #img = cv2.imread('/home/mamoon/Downloads/img2.jpg') 
    # Display original image
    #cv2.imshow('Original', cv2.resize(img,(800,400)))
    #cv2.waitKey(2000)

    # Convert to graycsale
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Blur the image for better edge detection
    img_blur = cv2.GaussianBlur(img_gray, (3,3), 0) 

    ret, thresh = cv2.threshold(img_blur, 95, 255,cv2.THRESH_BINARY_INV)
    contours, hierarchies = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    for i in contours:
        M = cv2.moments(i)
        if M['m00'] != 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.drawContours(img, [i], -1, (0, 255, 0), 2)
            cv2.circle(img, (cx, cy), 7, (0, 0, 255), -1)
            cv2.putText(img, "center", (cx - 20, cy - 20),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
            print(f"x: {cx} y: {cy}")
    cv2.imshow('Original', img)
    #cv2.imshow('Original', cv2.resize(img,(400,200)))
    cv2.waitKey(0)


def using_hsv():
    green=[(35, 40, 25), (70, 255, 255)]
    orange=[(75,180,25),(105,255,255)]
    red=[(75,180,25),(105,255,255)]
    yellow=[(75,180,25),(105,255,255)]
    blue=[(75,180,25),(105,255,255)]
    #black=[(75,180,25),(105,255,255)]

    port=-1 #cam port for webcam
    video=cv2.VideoCapture(port)
    rslt,img=video.read()
    #480,640,3
    print(img.shape)
    cv2.imshow("camera_view",cv2.resize(img,(400,200)))
    cv2.waitKey(0)
    img=rotate_image(img)
    cv2.imshow("rotated",cv2.resize(img,(400,200)))
    cv2.waitKey(0)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    #Create a mask for the object by selecting a possible range of HSV colors that the object can have:
    mask = cv2.inRange(hsv, green[0],green[1])
    #slice the object
    imask = mask>0
    #print(imask.shape)
    img_msk = np.zeros_like(img, np.uint8)
    img_msk[imask] = img[imask]
    #orange=np.clip(orange, 0, 255)
    print(img_msk[imask].shape)
    new_image=cv2.cvtColor(img_msk,cv2.COLOR_HSV2RGB)
    #new_image=cv2.cvtColor(new_image,cv2.COLOR_BGR2RGB)
    cv2.imshow("detection",cv2.resize(img_msk,(400,200)))
    cv2.waitKey(0)
    #img_gray = cv2.cvtColor(new_image,cv2.COLOR_RGB2GRAY)
    # Blur the image for better edge detection
    #img_blur = cv2.GaussianBlur(img_gray, (3,3), 0) 
    #ret, thresh = cv2.threshold(img_blur, 100, 255,cv2.THRESH_BINARY_INV)
    #thresh= cv2.Canny(img_gray,0,200)
    contours, hierarchies = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    centers=[]
    for i in contours:
        M = cv2.moments(i)
        if M['m00'] >100:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            #cv2.drawContours(new_image, [i], -1, (0, 255, 0), 2)
            #cv2.circle(new_image, (cx, cy), 7, (0, 0, 255), -1)
            #cv2.putText(new_image, "center", (cx - 20, cy - 20),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
            print(f"x: {cx} y: {cy}")
            centers.append([cx,cy])
    centers=np.array(centers)
    print(centers.shape)
    avg_center=np.mean(centers,axis=0)
    print(avg_center)
    cv2.circle(new_image, (int(avg_center[0]),int(avg_center[1])), 7, (0, 0, 255), -1)
    cv2.imshow('Original', new_image)
    #cv2.imshow('Original', cv2.resize(img,(400,200)))
    cv2.waitKey(0)
    return avg_center

def video():
    while True:
        port=-1 #cam port for webcam
        video=cv2.VideoCapture(port)
        rslt,img=video.read()
        if rslt==False:
            continue
        rotated=rotate_image(img)
        cv2.imshow("video",rotated)
        q = cv2.waitKey(1)
        if q == ord("q"):
            print(rotated.shape)
            break


if __name__=="__main__":
    #test()
    #getimage()
    #using_hsv()
    #focus()
    #video()
    #detect_center()
    center=using_hsv()
    cartesian_coordiantes= pixel_to_cartesian(center)
    print("cartesian",cartesian_coordiantes)
    '''try:
        getimage()
    except rospy.ROSInterruptException:
        pass'''
