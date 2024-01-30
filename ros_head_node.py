# 模块导入
import rospy
from jacobi_head_driver_plus import Steering_gear
from std_msgs.msg import String

class ROSNode:
    def __init__(self):
        self.lobot = Steering_gear
        # 初始化ros节点
        rospy.init_node('jacobi_head_node', anonymous=True)

        # 创建订阅者
        rospy.Subscriber('servo_command', String, self.servo_command_callback)

        # 创建ros话题发布者
        self.result_pud = rospy.Publisher('jicobi/jsr/head', String,queue_size=10)

    def servo_command_callback(self, data):
        if command =='move_servo1':
            self.lobot.moveServo(1,90,1000)
        else:
            self.lobot.moveServo(7,45,1000)

    def run(self):
        rate = rospy.Rate(10)  # 设置ros节点的循环频率

        while not rospy.is_shutdown():
            result = "Some resul="
            self.result_pud.publish(result)
            rate.sleep()

if __name__ == '__main__':
    try:
        ros_node = ROSNode()
        ros_node.run()
    except rospy.ROSInitException:
        pass

