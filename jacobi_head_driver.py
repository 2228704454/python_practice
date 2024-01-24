# 导入模块
import rospy  # 提供与ros交互通信的功能
import serial
#  from serial import SerialExceptin  # 导入serial中的SerialExceptin类，处理串口通讯异常
from serial import SerialException
from time import sleep  # 导入time模块中的sleep函数，用于添加时延
from std_msgs.msg import String  # 导入所需模块和消息类型


# FRAME_HEADER =  0x55,  # 帧头
# CMD_SERVO_MOVE = 0x03,  # 舵机移动指令
# CMD_ACTION_GROUP_RUN = 0x06,  # 运行动作组指令
# CMD_ACTION_GROUP_STOP = 0x07,  # 停止动作组运行指令
# CMD_ACTION_GROUP_SPEED = 0x0B, # 设置动作组运行速度指令
# CMD_GET_BATTERY_VOLTAGE = 0x0F  # 获得电池电压指令




# 定义路舵机类
class Steering_gear:  # 定义类
    def __init__(self) -> object:  # 构造函数
        self.numOfActinGroupRunning = 0xFF  # 定义类的变量，并且初始化为十六进制值0xFF
        self.actionGrouRunTimes = 0  # 定义类的动作运行时间变量，并初始化值为整数0
        self.isRunning = False  # 定义类的运行变量，初始化值为布尔值为假
        self.batteryVoltage = 0  # 定义类的电池电压变量，初始化值为整数0
        
    def moveServo(self, servoID, position, time):  # 构造函数 并定义形参舵机ID，目标位置，及转动时间
        if servoID > 31 or time <= 0:  # 判断如果时间舵机ID大于31或者时间小于等于0，则返回函数不执行下面代码
            return

        buf = bytearray(11)  # 创建一个长度为11的字节数组buf，用于存储要发送的数据包
        buf[0] = 0x55
        buf[1] = 0x55  # buf字节数组的第一和第二个元素为十六进制值0x55，这是数据包的帧头
        buf[2] = 8  # 表示该buf数据包的长度
        buf[3] = 3  # 定义第四个数值为整数值3，表示舵机移动指令
        buf[4] = 1  # 表示控制舵机个数
        buf[5] = time & 0xFF
        buf[6] = (time >> 8) & 0xFF  # 将转动时间拆分为低八位与高八位
        buf[7] = servoID  # 将第七个元素设置为舵机ID
        buf[8] = position & 0xFF
        buf[9] = (position >> 8) & 0xFF  # 将position拆分为低八位和高八位 
        self.serial.write(buf) 
        # serialX.write(buf, 10)
        # 在这里添加相应的串口通讯代码

    def moveServos(self, servoID, position, time):  # 构造函数 并定义形参舵机ID，目标位置，及转动时间
        if servoID > 31 or time <= 0:  # 判断如果时间舵机ID大于31或者时间小于等于0，则返回函数不执行下面代码
            return

        buf = bytearray(11)  # 创建一个长度为11的字节数组buf，用于存储要发送的数据包
        buf[0] = 0x55
        buf[1] = 0x55  # buf字节数组的第一和第二个元素为十六进制值0x55，这是数据包的帧头
        buf[2] = 8  # 表示该buf数据包的长度
        buf[3] = 3  # 定义第四个数值为整数值3，表示舵机移动指令
        buf[4] = 1  # 表示控制舵机个数
        buf[5] = time & 0xFF
        buf[6] = (time >> 8) & 0xFF  # 将转动时间拆分为低八位与高八位
        buf[7] = servoID  # 将第七个元素设置为舵机ID
        buf[8] = position & 0xFF
        buf[9] = (position >> 8) & 0xFF  # 将position拆分为低八位和高八位 
        self.serial.write(buf) 

    def OpenSerialPot(self, port, baud_rate):  # 构造一个打开串口OpenSerialPot的函数，里面定义两个形参port, baud_rate
        try:  # try语法，异常情况处理
            self.serial = serial.Serial(port, baud_rate)  # 实例化一个对象serial赋值等于Serial(port, baud_rate)
        except SerialException as e:  # 异常情况处理1出现打开失败
            rospy.logerr("打开失败")
            raise e  # 返回上级调用
    
    # def Sports(self, servo_id, *args):  # 构造一个运动控制的函数 Sports，里面定义servo_id, *args两个形参
    #       将变量command输入到实例化对象serial中，实现将指令传输给串口


def Check_limit(motor1, motor2):  # 构造一个检查限位的函数Check_limit，里面定义两个形参motor1, motor2
    if motor2 < -36 or motor2 > 90 or motor1 < -130 or motor1 > 130:  # 判断如果当motor2小于反方向的36度或者当motor2大于90度或者motor1小于反方向的130度或者大于130度
        return False  # 满足以上任意一个条件，返回值返回假
    return True  # 反之返回为真


if __name__ == '__main__':  # 构造main函数，表示以下代码块只在运行文件时执行，作为模块导入时不执行
    rospy.init_node("send")  # 初始化ROS节点名称为send
    port = "/dev/ttyUSB0"  # 设置串口名称
    baud_rate = 9600  # 设置波特率
    # print(rospy.init_node("send"))

    lobot = Steering_gear() # 创建一个 Steering_gear对象并赋值给lobot

    try:  # try语法异常情况处理
        lobot.OpenSerialPot(port, baud_rate)  # 调用lobot对象中的OpenSerialPot函数，打开串口
    except SerialException:  # 如果出现异常情况
        exit(-2)  # 退出整个程序，并返回值-2

    reat = rospy.Rate(5)  # 创建一个对象rospy.Rate赋值给reat，控制频率为5Hz
    while not rospy.is_shutdown():  # 创建循环，如果ros节点没有被关闭，则以下代码一直进行循环
        lobot.moveServo(7, 1500, 1000)  # 调用lobot对象中的Sports函数，传递实参
        lobot.moveServos(1, 1000,1000)  
        rospy.spin()  # 处理一次ros消息
        reat.sleep()  # 保持循环频率为5Hz


