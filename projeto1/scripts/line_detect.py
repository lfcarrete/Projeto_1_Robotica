#!/usr/bin/env python
# BEGIN ALL
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Vector3, Pose, Vector3Stamped
import visao_module
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
from tf import transformations
import time

goal = ["blue",11,"cat"]
image = None
media = 0
centro = 0
maior_area = 0
perto = False
dist = 0
bateu = False
x = None
y = None
x_inicial = None 
y_inicial = None
contador = 0
achou = False
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
        print("Direita")
    if (media[0] < centro[0] - tolerancia):
        vel = Twist(Vector3(0,0,0), Vector3(0,0,0.1))
        cmd_vel_pub.publish(vel)
        print("Esquerda")
    if (media[0] > centro[0]  - tolerancia and media[0] < centro[0]  + tolerancia ):
        vel = Twist(Vector3(0.3,0,0), Vector3(0,0,0))
        cmd_vel_pub.publish(vel)

def scaneou(dados):
    global dist
    dist = dados.ranges[359]
    #print(dist)
    
def odometria(dados):
    global x
    global y
    global x_inicial
    global y_inicial
    global contador
    global perto

    x = dados.pose.pose.position.x
    y = dados.pose.pose.position.y

    if perto and contador == 0:
        x_inicial = x
        y_inicial = y
        contador += 1


if __name__ == '__main__':
    rospy.init_node('follower')


    image_sub = rospy.Subscriber('/camera/rgb/image_raw', 
                                      Image, image_callback)
    cmd_vel_pub = rospy.Publisher('/cmd_vel',
                                        Twist, queue_size=3)
    twist = Twist()

    recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)

    odom = rospy.Subscriber("/odom", Odometry, odometria) #Baseado no codigo do gabarito da questao 3 da prova 1


    while not rospy.is_shutdown():

        if image is not None:
            cv2.imshow("window", image)
            media, centro, maior_area = visao_module.identifica_cor(image,goal[0])
            
            cv2.waitKey(3)
    
            #print(x_inicial)
            #print(y_inicial)
            print(x)
            print(y)
            if dist <= 0.25 and dist > 0:
                bateu = True
                print("Bateu")
            if maior_area > 3000:
                perto = True
                print("perto")
            if media[0] < 400 and media[0] > 0 and perto and bateu == False:
                centraliza_creeper()
            if bateu and achou == False:
                while x > x_inicial - 0.2 and y > y_inicial - 0.2:
                    vel = Twist(Vector3(-0.2,0,0),Vector3(0,0,0))
                    cmd_vel_pub.publish(vel)
                    print("Dentro do while")
                vel = Twist(Vector3(0,0,0),Vector3(0,0,0))
                cmd_vel_pub.publish(vel)
                achou = True
                
                



                
                
# END ALL