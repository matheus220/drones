#! /usr/bin/env python
import rospy
import numpy as np
import quaternion

from formation_control_lib.msg import Formation
from formation_control_lib.msg import FormationLink
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from gazebo_msgs.msg import ModelStates

from qualisys.msg import Subject

class theFormation:
    def __init__(self):
        self.drone4_found = False
        self.drone5_found = False
        self.drone6_found = False
        self._drones = ['drone4','drone5','drone6']
        self._from = [self._drones[0],self._drones[0],self._drones[1],self._drones[2]]
        self._to = [self._drones[1],self._drones[2],self._drones[0],self._drones[1]]
        self._embeddings = np.array([[0,0,0],[1,0,0],[0,1,0]])
        print self._from
        print self._to
        print self._embeddings

        rospy.init_node('formation_generator', anonymous=True)
        self._pub = rospy.Publisher('bearings',Formation,queue_size=1)
        self._pub_gazebo = rospy.Publisher('gazebo/model_states_fake',ModelStates,queue_size=1)
        self._pub_real_drone_pose = rospy.Publisher('/uav1/mavros/mocap/pose',PoseStamped,queue_size=1)
        self._sub = rospy.Subscriber("/qualisys/drone4", Subject, self.callback_d4)
        self._sub = rospy.Subscriber("/qualisys/drone5", Subject, self.callback_d5)
        self._sub = rospy.Subscriber("/qualisys/drone6", Subject, self.callback_d6)

        self._msg = Formation()
        self._gazebo_poses = ModelStates()

        self._poses = [Pose(), Pose(), Pose()]
        self._poses[0].orientation.w = 1
        self._poses[1].orientation.w = 1
        self._poses[2].orientation.w = 1


    def spin(self):
        rate = rospy.Rate(5)
        print 'ROS Node spinning at 5 Hz'

        while not rospy.is_shutdown():
            if(self.drone4_found and self.drone5_found and self.drone6_found):
                self.publish_bearings()
                self.publish_poses()

            rate.sleep()

        rospy.on_shutdown(self.node_shutdown())

    def publish_poses(self):
        self._gazebo_poses.name = ['iris_1','iris_2','iris_3']
        self._gazebo_poses.pose = self._poses
        self._gazebo_poses.twist = [Twist(), Twist(), Twist()]
        self._pub_gazebo.publish(self._gazebo_poses)

    def publish_bearings(self):
        #print("calculating bearings")
        self._msg.header.stamp = rospy.Time.now()
        self._msg.drones = self._drones
        self._msg.links = []
        for i in range(len(self._drones)): # loop through list of drones
            self._msg.links.append(FormationLink())
            self._msg.links[i].drone_name = self._drones[i]
            for j in range(len(self._from)): # loop through list of drones observing
                if self._from[j] == self._drones[i]:
                    self._msg.links[i].targets.append(self._to[j])
                    target_index = self._drones.index(self._to[j])
                    from_index = self._drones.index(self._from[j])
                    bearing = Vector3()

                    dx = self._poses[target_index].position.x - self._poses[from_index].position.x
                    dy = self._poses[target_index].position.y - self._poses[from_index].position.y
                    dz = self._poses[target_index].position.z - self._poses[from_index].position.z
                    vect = np.array([[dx],[dy],[dz]])
                    distance = Float64()
                    distance.data = np.linalg.norm(vect)
                    self._msg.links[i].distances.append(distance)
                    vect = vect/np.linalg.norm(vect)

                    # find world frame bearing vector
                    w = self._poses[from_index].orientation.w
                    x = self._poses[from_index].orientation.x
                    y = self._poses[from_index].orientation.y
                    z = self._poses[from_index].orientation.z

                    # find local frame bearing vector
                    q = np.array([quaternion.quaternion(w,x,y,z)])
                    R = quaternion.as_rotation_matrix(q)
                    RT = np.transpose(R).reshape(1,3,3)
                    vect_rotatated = np.matmul(RT, vect)

                    bearing.x = vect_rotatated[0][0]
                    bearing.y = vect_rotatated[0][1]
                    bearing.z = vect_rotatated[0][2]
                    self._msg.links[i].bearings.append(bearing)

        self._pub.publish(self._msg)

        ## Shutdown condition for node
    def node_shutdown(self):
        print 'shutting down formation generator node...'

    def callback_d4(self,data):
        self.drone4_found = True
        tmp_msg = Pose()
        tmp_msg.position = data.position
        tmp_msg.orientation = data.orientation
        self._poses[0] = tmp_msg

    def callback_d5(self,data):
        self.drone5_found = True
        tmp_msg = Pose()
        tmp_msg.position = data.position
        tmp_msg.orientation = data.orientation
        self._poses[1] = tmp_msg

    def callback_d6(self,data):
        self.drone6_found = True
        tmp_msg = Pose()
        tmp_msg.position = data.position
        tmp_msg.orientation = data.orientation
        self._poses[2] = tmp_msg
        mocap_pose = PoseStamped()
        mocap_pose.header.stamp = rospy.Time.now()
        mocap_pose.pose.position = tmp_msg.position
        mocap_pose.pose.orientation = tmp_msg.orientation
        self._pub_real_drone_pose.publish(mocap_pose)

## main function
if __name__ == '__main__':
    try:
        f = theFormation()
        f.spin()
    except rospy.ROSInterruptException:
        print 'Formation generation node couldnt initialize'
        pass

