#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time
from std_srvs.srv import Empty

def poseCallback(pose_message):
    global x
    global y, yaw
    x= pose_message.x
    y= pose_message.y
    yaw = pose_message.theta

    #print "pose callback"
    #print ('x = {}'.format(pose_message.x)) #new in python 3
    #print ('y = %f' %pose_message.y) #used in python 2
    #print ('yaw = {}'.format(pose_message.theta)) #new in python 3


def move(velocity_publisher, speed, distance, is_forward):
        #declare a Twist message to send velocity commands
        velocity_message = Twist()
        #get current location 
        global x, y
        x0=x
        y0=y

        if (is_forward):
            velocity_message.linear.x =abs(speed)
        else:
        	velocity_message.linear.x =-abs(speed)

        distance_moved = 0.0
        loop_rate = rospy.Rate(10) # we publish the velocity at 10 Hz (10 times a second)    
        
        while True :
                rospy.loginfo("Turtlesim moves forwards")
                velocity_publisher.publish(velocity_message)

                loop_rate.sleep()
                
                distance_moved = abs(math.sqrt(((x-x0) ** 2) + ((y-y0) ** 2)))
                print  (distance_moved)
                print(x)
                if  not (distance_moved<distance):
                    rospy.loginfo("reached")
                    break
        
        #finally, stop the robot when the distance is moved
        velocity_message.linear.x =0
        velocity_publisher.publish(velocity_message)
    
def rotate (velocity_publisher, angular_speed_degree, relative_angle_degree, clockwise):
    
    velocity_message = Twist()

    angular_speed=math.radians(abs(angular_speed_degree))

    if (clockwise):
        velocity_message.angular.z =-abs(angular_speed)
    else:
        velocity_message.angular.z =abs(angular_speed)

    angle_moved = 0.0
    loop_rate = rospy.Rate(10) # we publish the velocity at 10 Hz (10 times a second)    
    cmd_vel_topic='/turtle1/cmd_vel'
    velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

    t0 = rospy.Time.now().to_sec()

    while True :
        rospy.loginfo("Turtlesim rotates")
        velocity_publisher.publish(velocity_message)

        t1 = rospy.Time.now().to_sec()
        current_angle_degree = (t1-t0)*angular_speed_degree
        loop_rate.sleep()


                       
        if  (current_angle_degree>relative_angle_degree):
            rospy.loginfo("reached")
            break

    #finally, stop the robot when the distance is moved
    velocity_message.angular.z =0
    velocity_publisher.publish(velocity_message)


def go_to_goal(velocity_publisher, x_goal, y_goal):
    global x
    global y, yaw

    velocity_message = Twist()

    while (True):
        K_linear = 0.5 
        distance = abs(math.sqrt(((x_goal-x) ** 2) + ((y_goal-y) ** 2)))

        linear_speed = distance * K_linear

        K_angular = 4.0
        desired_angle_goal = math.atan2(y_goal-y, x_goal-x)
        angular_speed = (desired_angle_goal-yaw)*K_angular

        velocity_message.linear.x = linear_speed
        velocity_message.angular.z = angular_speed

        velocity_publisher.publish(velocity_message)
        print ('x=', x, ', y=',y, ', distance to goal: ', distance)

        if (distance <0.01):
            break

def setDesiredOrientation(publisher, speed_in_degree, desired_angle_degree):
    relative_angle_radians = math.radians(desired_angle_degree) - yaw
    clockwise=0
    if relative_angle_radians < 0:
        clockwise = 1
    else:
        clockwise = 0
    print ("relative_angle_radians: ",math.degrees(relative_angle_radians))
    print ("desired_angle_degree: ",desired_angle_degree)
    rotate(publisher, speed_in_degree,math.degrees(abs(relative_angle_radians)), clockwise)
    
    
'''#############################################
This is the only defined function which is uncommented in the code. By trial it seems that the size (length of the turtle square window) is 11 units. In our case we will use 4 segments (4 in x or y direction which will give use 16 squares in the window)
#############################################'''

def gridClean(publisher):#ger
	size=11
	segments=4
	squareNum=[1,5,9,13,14,15,16,12,8,4,3,7,11,10,6,2] #the sequence of our squares. We start from square number 1 then 5 and so on. 
	xy=xandyCoordinates(size,segments,squareNum) #we will get a generator object
	xy=list(xy) #we will convert the object to a list. The list will be a two-dimensional list the first dimension is for the square sequence and the second is for the x and y values.
	
	setDesiredOrientation(velocity_publisher, 30, angleFromCoordinates([5.5,5.5],xy[0]))
	go_to_goal(velocity_publisher, xy[0][0], xy[0][1])
	clear()
	
	
	
	for i in range(len(xy)):
	
		spiralClean(velocity_publisher, wk=19, rk=2,rate=12,incrementSpeed=0.5,xc=xy[i][0],yc=xy[i][1],size=size,segments=segments,threshold=0.45)
		
		if (i!=len(xy)-1):
			setDesiredOrientation(velocity_publisher, 30, angleFromCoordinates([x,y],xy[i+1]))
			go_to_goal(velocity_publisher, xy[i+1][0], xy[i+1][1])

		
	setDesiredOrientation(velocity_publisher, 30, angleFromCoordinates([x,y],xy[i]))
	go_to_goal(velocity_publisher, xy[i][0], xy[i][1])
	setDesiredOrientation(velocity_publisher, 30, 0)



'''#############################################
The main modification in this function is that we have controlled the ranges (in the while loop brackets) for any x and y coordinates we will have a square range. Notice that we have also added a threshold just in case we do not want our turtle to hit the wall and to have more control of the cleaning area. The second modification is that we have added more parameter in the function. It is more convenient to adjust the spiral function from its arguments and try to see the result from one lie instead of scrolling up and down in the code. In general if we increase the rate the error will be less but in this function it is a little tricky because if we increase the rate we will also increase the linear acceleration the (the speed rk will grow in every loop). Of course, we could set the incrementSpeed argument to zero but that will give us a slow non convenient spiral.   
#############################################'''
def spiralClean(velocity_publisher, wk, rk,rate,incrementSpeed,xc,yc,size,segments,threshold):
    vel_msg = Twist()
    loop_rate = rospy.Rate(rate)
 
    while((xc-size/(segments*2)+threshold<x<xc+size/(segments*2)+threshold) and (yc-size/(segments*2)+threshold<y<yc+size/(segments*2)+threshold)):
        rk=rk+incrementSpeed
        vel_msg.linear.x =rk
        vel_msg.linear.y =0
        vel_msg.linear.z =0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z =wk
        velocity_publisher.publish(vel_msg)
        loop_rate.sleep()
 
    vel_msg.linear.x = 0
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)

   
'''#############################################
The purpose of this function is to clean the trace from the turtle.
#############################################'''
def clear():
	
	try:
		clearing=rospy.ServiceProxy('/clear', Empty) #creating the service client 
		clearing()
	
	except rospy.ServiceException as e:
        	print("Service call failed: %s"%e)

        	
'''#############################################
This function will give us all the x and y coordinates of the squares in the turtle window (the coordinates in the center of the square). Note this function is the most important function for our approach to solving this problem. Therefore, all the variables and our approach are explained in detail in a graph in the hands-on-2.pdf.
#############################################'''
def xandyCoordinates(size,segments,squareNum):

    for i in range(len(squareNum)):
        squareX=squareNum[i]%segments
        squareY=squareNum[i]//segments
        if squareX==0:
            squareX=segments
            squareY=squareY-1
        xc=size/(segments*2)+(squareX-1)*(size/segments)
        yc=size/(segments*2)+squareY*(size/segments)
        result=[xc,yc]
        yield result          	

        	


'''#############################################
This function will take two points and will return the angle in degrees (the first parameter point will be as the zeroes coordinates). Note that it has been taken under consideration if the angle is 90 or -90.
#############################################'''
def angleFromCoordinates(xy1,xy2):
    
    if (abs(xy2[0]-xy1[0])<0.01 and xy2[1]-xy1[1]>0):

        return 90

    elif(abs(xy2[0]-xy1[0])<0.01 and xy2[1]-xy1[1]<0):
        return -90

    angle=math.degrees(math.atan2((xy2[1]-xy1[1]),(xy2[0]-xy1[0])))
    return angle





if __name__ == '__main__':
    try:
        
        rospy.init_node('turtlesim_motion_pose', anonymous=True)

        #declare velocity publisher
        cmd_vel_topic='/turtle1/cmd_vel'
        velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
        
        position_topic = "/turtle1/pose"
        pose_subscriber = rospy.Subscriber(position_topic, Pose, poseCallback) 
        time.sleep(2)

        #move(velocity_publisher, 1.0, 9.0, True)
        #rotate(velocity_publisher, 30, 90, True)
        #go_to_goal(velocity_publisher, 2.0, 1.5)
        #setDesiredOrientation(velocity_publisher, 30, 90)
        #spiralClean(velocity_publisher, 4, 0)
        gridClean(velocity_publisher)
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")