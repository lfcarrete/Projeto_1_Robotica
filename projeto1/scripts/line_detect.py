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
x_final = None
y_final = None
contador = 0
max_angular = math.pi/8
alfa = -1

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
    global x_final
    global y_final
    global y_inicial
    global contador
    global alfa

    x = dados.pose.pose.position.x
    y = dados.pose.pose.position.y

    quant = dados.pose.pose.orientation
    lista = [quant.x,quant.y,quant.z,quant.w]
    angulos_rad = transformations.euler_from_quaternion(lista)
    angulos = numpy.degrees(angulos_rad)
    alfa = angulos_rad[2]

    if contador == 0:
        x_inicial = x
        y_inicial = y
        contador += 1
    if bateu == False:
        x_final = x
        y_final = y

    

def calcula_dist(x,y):
    hipo = math.sqrt(math.pow(x,2) + math.pow(y,2))
    return hipo

def calcula_angulo(alfa, x,y):
    beta = math.atan((y/x))
    angulo_total = beta + math.pi - alfa
    return angulo_total

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
            
            if dist <= 0.25 and dist > 0:
                bateu = True
                print("Bateu")
            if maior_area > 3000:
                perto = True
                print("perto")
            if media[0] < 400 and media[0] > 0 and perto and bateu == False:
                centraliza_creeper()
            if bateu:
                ang = calcula_angulo(alfa, (x_final - x_inicial), (y_final - y_inicial))
                dist = calcula_dist((x_final - x_inicial), (y_final-y_inicial))
                
                vel_rot = Twist(Vector3(0,0,0),Vector3(0,0,max_angular))
                vel_trans = Twist(Vector3(0.2,0,0),Vector3(0,0,0))
                zero = Twist(Vector3(0,0,0),Vector3(0,0,0))
        
                sleep_rot = abs(ang/max_angular)
                sleep_trans = abs(dist/0.2)
                
                cmd_vel_pub.publish(zero)
                print("SLEEP")
                rospy.sleep(10.0)
                

                
                

                print("AAAA")

                #cmd_vel_pub.publish(Twist(Vector3(0,0,0),Vector3(0,0,0.3)))
                #rospy.sleep(30)
                
# END ALL