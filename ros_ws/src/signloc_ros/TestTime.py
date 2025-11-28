import rosbag
from graphloc_msgs.msg import Sign as gSign
from graphloc_msgs.msg import SignElement as gSignElement
from signloc_msgs.msg import Sign, SignElement
import rospy


def change_topic_pkg(input_bag_path, output_bag_path, topic_name="/signs"):

    with rosbag.Bag(input_bag_path, 'r') as in_bag, rosbag.Bag(output_bag_path, 'w') as out_bag:
     for topic, msg, t in in_bag.read_messages():
         if topic == topic_name and hasattr(msg, 'header') and msg.header.stamp:

            sign_msg = Sign()
            sign_msg.header = msg.header
            elements = []
            for e in msg.elements:
                sign_elemnts_msg = SignElement()
                sign_elemnts_msg.place = e.place
                sign_elemnts_msg.directions = e.directions
                sign_elemnts_msg.probs = e.probs
                elements.append(sign_elemnts_msg)
            sign_msg.elements = elements

            out_bag.write(topic, sign_msg, t)
            cnt+= 1
         else:
            out_bag.write(topic, msg, t=t)



def main():
    change_topic_pkg(input_bag_path="/ros_ws/signs.bag", output_bag_path="/ros_ws/signs2.bag", topic_name="/signs")

if __name__ == '__main__':
    main()