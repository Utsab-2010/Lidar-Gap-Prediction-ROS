import rospy
from std_msgs.msg import String

class FixedRateSubscriber:
    def __init__(self):
        rospy.init_node('fixed_rate_subscriber')
        self.latest_message = None
        self.subscriber = rospy.Subscriber('topic_name', String, self.callback)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.process_message)  # 10 Hz

    def callback(self, msg):
        self.latest_message = msg  # Store the latest message

    def process_message(self, event):
        if self.latest_message:
            rospy.loginfo(f'Processing message: {self.latest_message.data}')
            # Add processing logic here

if __name__ == '__main__':
    try:
        node = FixedRateSubscriber()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
