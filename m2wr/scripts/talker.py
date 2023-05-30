import rospy
from geometry_msgs.msg import Twist, Pose, Point
from nav_msgs.msg import Odometry
import math

x, y = 0, 0


def callback_func(msg):
    global x, y
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    # print(x, y)
        
    
def forward(distance):
    vel = Twist()
    rate = rospy.Rate(100)
    vel.linear.x = -0.2
    global x, y
    x0, y0 = x, y 
    dist_now = 0
    while dist_now < distance:
        dist_now = (((x-x0)**2)+((y-y0)**2))**0.5
        pub.publish(vel)
        print(dist_now)
        # rospy.spin()
        rate.sleep()
    
    vel.linear.x = 0
    pub.publish(vel)

def rotate(angle):
    
    rate = rospy.Rate(1000)
    vel = Twist()
    vel.linear.x = 0
    vel.linear.y = 0
    vel.linear.z = 0
    
    ang_speed = 0.3
    
    vel.angular.x = 0
    vel.angular.y = 0
    vel.angular.z = ang_speed
    
    cur_angle = 0.0
    start_time = rospy.get_time()
    print((start_time))
    while (cur_angle) < float(angle):
        cur_angle = ang_speed * abs(start_time - rospy.get_time())
        pub.publish(vel)
        # rospy.spin()
        rate.sleep()
        print(cur_angle)
    vel.angular.z = 0
    pub.publish(vel)
        

if __name__ == "__main__":
    try:
        pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        sub = rospy.Subscriber('odom', Odometry, callback=callback_func)
        rospy.init_node('talker', anonymous=True) 
        for item in range(4):
            forward(0.5)
            rotate(math.radians(117))
        # for item in range(3):
        #     forward(1)
        #     rotate(math.radians(140))
    except Exception as ex:
        print(ex)