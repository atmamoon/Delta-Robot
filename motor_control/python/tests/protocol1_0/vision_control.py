#!/usr/bin/env python
import urllib.request as ur
#import rospy
#from rospy.numpy_msg import numpy_msg

#from sensor_msgs.msg import Image as SensorImage
import cv2
#from cv_bridge import CvBridge
import numpy as np
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
    cv2.waitKey(0)

    cv2.destroyAllWindows()

def detect_center():
    port=2 #cam port for webcam
    video=cv2.VideoCapture(port)
    rslt,img=video.read()
    cv2.imshow("camera_view",cv2.resize(img,(800,400)))
    cv2.waitKey(0)
    # Read the original image
    #img = cv2.imread('/home/mamoon/Downloads/test4.jpg')
    #img = cv2.imread('/home/mamoon/Desktop/engineering/test_img.jpg')
    #img = cv2.imread('/home/mamoon/Downloads/img2.jpg') 
    # Display original image
    cv2.imshow('Original', cv2.resize(img,(800,400)))
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
    cv2.imshow('Original', cv2.resize(img,(800,800)))
    cv2.waitKey(0)
 

if __name__=="__main__":
    #test()
    #getimage()
    detect_center()
    '''try:
        getimage()
    except rospy.ROSInterruptException:
        pass'''
