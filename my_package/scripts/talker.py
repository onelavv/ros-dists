#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    # rate = rospy.Rate(12) # 10hz
    # while not rospy.is_shutdown():
    #     hello_str = "Мазда, жигули, мерсендес, бнв, шкода"
    #     rospy.loginfo(hello_str)
    #     pub.publish(hello_str)
    #     rate.sleep()
    for item in range(5):
        pub.publish("Фамилия 01.01.1970")

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
