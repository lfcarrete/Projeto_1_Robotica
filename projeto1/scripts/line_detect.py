#!/usr/bin/env python
# BEGIN ALL
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Vector3, Pose, Vector3Stamped
import visao_module

goal = ["green",11,"cat"]
image = None
media = 0
centro = 0
maior_area = 0
perto = False
def image_callback(msg):
    
    global image
    global media
    global centro
    global maior_area

    image = cv_bridge.CvBridge().imgmsg_to_cv2(msg,desired_encoding='bgr8')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_yellow = numpy.array([ 10,  10,  10])
    upper_yellow = numpy.array([255, 255, 250])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    

    h, w, d = image.shape
    search_top = 3*h/4
    search_bot = 3*h/4 + 20
    mask[0:search_top, 0:w] = 0
    mask[search_bot:h, 0:w] = 0
    M = cv2.moments(mask)

    if M['m00'] > 0:
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        cv2.circle(image, (cx, cy), 20, (0,0,255), -1)

        # BEGIN CONTROL
        err = cx - w/2
        twist.linear.x = 0.1
        twist.angular.z = -float(err) / 200
        cmd_vel_pub.publish(twist)

        # END CONTROL

def centraliza_creeper():
    global perto

    tolerancia = 25
    if (media[0] > centro[0] + tolerancia):
    	vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.1))
        cmd_vel_pub.publish(vel)
        print("esquerda")
    if (media[0] < centro[0] - tolerancia):
        vel = Twist(Vector3(0,0,0), Vector3(0,0,0.1))
        cmd_vel_pub.publish(vel)
        print("direita")
    if (media[0] > centro[0]  - tolerancia and media[0] < centro[0]  + tolerancia ):
        vel = Twist(Vector3(0.3,0,0), Vector3(0,0,0))
        cmd_vel_pub.publish(vel)



if __name__ == '__main__':
    rospy.init_node('follower')
    image_sub = rospy.Subscriber('/camera/rgb/image_raw', 
                                      Image, image_callback)
    cmd_vel_pub = rospy.Publisher('/cmd_vel',
                                        Twist, queue_size=1)
    twist = Twist()


    while not rospy.is_shutdown():

        if image is not None:
            cv2.imshow("window", image)
            media, centro, maior_area = visao_module.identifica_cor(image,goal[0])
            
            cv2.waitKey(3)
            #print("media: {}".format(media))
            #print("centro: {}".format(centro))
            #print("maior_area: {}".format(maior_area))
            if maior_area > 3000:
                perto = True
                print("perto")
            if media[0] < 400 and media[0] > 0 and perto:
                centraliza_creeper()

                
# END ALL