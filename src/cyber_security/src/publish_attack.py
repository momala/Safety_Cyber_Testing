import argparse
import rospy
import time
from cyber_security.msg import CyberLidarFakePoints



rospy.init_node('talker_cyber', anonymous=True)
pub = rospy.Publisher('/cyber_attack_param', CyberLidarFakePoints, queue_size=10)
 
def publish_message(status,points,freq,duration):
    """
    This function publish a message to maintain an attack
    
    Args: 
    status = True or False (enable or disable the attack)
    points = Number of the false point
    freq = frequency of the false point publisher
    radius = Radius of the false points around the direction  Duration of the attack
    
    """
    message = CyberLidarFakePoints()
    message.start_distance = 2 # initial point of noise wrt lidar
    message.end_distance = 15 # final point of noise wrt lidar
    message.is_attack = status
    message.radius = duration
    message.number_of_points = points
    message.frequency = freq
    time.sleep(0.2)
    pub.publish(message)



def enable_cyber_attack():

    parser = argparse.ArgumentParser(description="Arguments to Maintaining the cyber-attack",
                                 formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("-a", "--attack", type=str, help="Enable or Disable the attack")
    parser.add_argument("-p", "--points", type=int, default=500, help="Number of fake points generated")
    parser.add_argument("-f", "--frequency",type=float, default=10, help="Publisher Frequency")
    parser.add_argument("-d", "--duration",type=float, default=1, help="Radius of the false points around the direction")
    args = parser.parse_args()
    config = vars(args)
    print(args.attack == "True",args.points,args.frequency,args.duration)
    print("----------Starting/Stopping Attack----------")
    publish_message(args.attack == "static",args.points,args.frequency,args.duration)



if __name__ == '__main__':
    try:
        enable_cyber_attack()
    except rospy.ROSInterruptException:
        pass