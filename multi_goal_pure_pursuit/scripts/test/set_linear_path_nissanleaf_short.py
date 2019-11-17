import rospy
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState

def main():
    rospy.init_node('datacollector_lateral_error')
    print("Reseting poses, for linear path validation")

    state_msg = ModelState()
    state_msg.model_name = 'nissanleaf'
    state_msg.pose.position.x = -47
    state_msg.pose.position.y = 52.8
    state_msg.pose.position.z = 0.431
    state_msg.pose.orientation.x = 0
    state_msg.pose.orientation.y = 0
    state_msg.pose.orientation.z = 0
    state_msg.pose.orientation.w = 1

    reset_simulation= rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    print(reset_simulation(state_msg))
    
    


if __name__=="__main__":
    main()