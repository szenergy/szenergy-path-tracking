import rospy

from std_msgs.msg import Float64, Int32

from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState

def main():
    rospy.init_node('datacollector_lateral_error')
    print("Reseting poses, monitoring lateral error")

    state_msg = ModelState()
    state_msg.model_name = 'nissanleaf'
    state_msg.pose.position.x = -4.0
    state_msg.pose.position.y = -30
    state_msg.pose.position.z = 0.3
    state_msg.pose.orientation.x = 0
    state_msg.pose.orientation.y = 0
    state_msg.pose.orientation.z = 0.707
    state_msg.pose.orientation.w = 0.707

    reset_simulation= rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    print(reset_simulation(state_msg))

if __name__=="__main__":
    main()