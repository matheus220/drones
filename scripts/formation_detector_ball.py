#! /usr/bin/env python
import rospy
from formation_control_lib.msg import Formation
from formation_control_lib.msg import FormationLink

class theFormation:
    def __init__(self):
        self.drone_found = False

        rospy.init_node('formation_detector_ball', anonymous=True)
        self._pub = rospy.Publisher('bearings',Formation,queue_size=1)
        self._sub = rospy.Subscriber("bearing_topic_1", FormationLink, self.callback)
        self._sub = rospy.Subscriber("bearing_topic_2", FormationLink, self.callback)
        self._sub = rospy.Subscriber("bearing_topic_3", FormationLink, self.callback)

        self._msg = Formation()

    def spin(self):
        rate = rospy.Rate(20)

        while not rospy.is_shutdown():
            if(self.drone_found):
                self._pub.publish(self._msg)
                self._msg = Formation()

            rate.sleep()

        rospy.on_shutdown(self.node_shutdown())

    ## Shutdown condition for node
    def node_shutdown(self):
        print 'shutting down formation generator node...'

    def callback(self,data):
        self.drone_found = True
        self._msg.drones.append(data.drone_name)
        self._msg.links.append(data)


## main function
if __name__ == '__main__':
    try:
        f = theFormation()
        f.spin()
    except rospy.ROSInterruptException:
        print 'Formation generation node couldnt initialize'
        pass

