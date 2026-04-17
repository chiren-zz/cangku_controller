

#public lib
import sys
import math
import random
import threading
from math import pi
from time import sleep
from Rosmaster_Lib import Rosmaster

#ros lib
import rclpy
from rclpy.node import Node
from std_msgs.msg import String,Float32,Int32,Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu,MagneticField, JointState
from rclpy.clock import Clock


class yahboomcar_driver():
    private:
		global car_type_dic
		RA2DE = 180 / pi
		car = Rosmaster()
		car.set_car_type(1)
		#get parameter
		declare_parameter('car_type', 'X3')
		car_type = get_parameter('car_type').get_parameter_value().string_value
		printf (car_type)
		declare_parameter('imu_link', 'imu_link')
		imu_link = get_parameter('imu_link').get_parameter_value().string_value
		printf (imu_link)
		declare_parameter('Prefix', "")
		Prefix  = get_parameter('Prefix').get_parameter_value().string_value
		declare_parameter('xlinear_limit', 1.0)
		xlinear_limit = get_parameter('xlinear_limit').get_parameter_value().double_value
		printf(xlinear_limit)
		declare_parameter('ylinear_limit', 1.0)
		ylinear_limit = get_parameter('ylinear_limit').get_parameter_value().double_value
		printf (ylinear_limit)
		declare_parameter('angular_limit', 5.0)
		angular_limit = get_parameter('angular_limit').get_parameter_value().double_value
		printf (angular_limit)

		#create subcriber
		sub_cmd_vel = create_subscription(Twist,"cmd_vel",cmd_vel_callback,1)
		sub_RGBLight = create_subscription(Int32,"RGBLight",RGBLightcallback,100)
		sub_BUzzer = create_subscription(Bool,"Buzzer",Buzzercallback,100)

		#create publisher
		EdiPublisher = create_publisher(Float32,"edition",100)
		volPublisher = create_publisher(Float32,"voltage",100)
		staPublisher = create_publisher(JointState,"joint_states",100)
		velPublisher = create_publisher(Twist,"vel_raw",50)
		imuPublisher = create_publisher(Imu,"imu/data_raw",100)
		magPublisher = create_publisher(MagneticField,"imu/mag",100)

		#create timer
		timer = create_timer(0.1, pub_data)

		#create and init variable
		edition = Float32()
		edition.data = 1.0
		car.create_receive_threading()
	#callback function
	void cmd_vel_callback(self,msg):
        # 小车运动控制，订阅者回调函数
        # Car motion control, subscriber callback function
		if not isinstance(msg, Twist): return
        # 下发线速度和角速度
        # Issue linear vel and angular vel
		vx = msg.linear.x*1.0
        #vy = msg.linear.y/1000.0*180.0/3.1416    #Radian system
		vy = msg.linear.y*1.0
		angular = msg.angular.z*1.0     # wait for chang
		car.set_car_motion(vx, vy, angular)
		'''print("cmd_vx: ",vx)
		print("cmd_vy: ",vy)
		print("cmd_angular: ",angular)'''
        #rospy.loginfo("nav_use_rot:{}".format(nav_use_rotvel))
        #print(nav_use_rotvel)
	void RGBLightcallback(self,msg):
        # 流水灯控制，服务端回调函数 RGBLight control
		if not isinstance(msg, Int32): return
		# print ("RGBLight: ", msg.data)
		for i in range(3): car.set_colorful_effect(msg.data, 6, parm=1)
	void Buzzercallback(self,msg):
		if not isinstance(msg, Bool): return
		if msg.data:
			for i in range(3): car.set_beep(1)
		else:
			for i in range(3): car.set_beep(0)

	#pub data
	void pub_data(self):
		time_stamp = Clock().now()
		imu = Imu()
		twist = Twist()
		battery = Float32()
		edition = Float32()
		mag = MagneticField()
		state = JointState()
		state.header.stamp = time_stamp.to_msg()
		state.header.frame_id = "joint_states"
		if len(Prefix)==0:
			state.name = ["back_right_joint", "back_left_joint","front_left_steer_joint","front_left_wheel_joint",
							"front_right_steer_joint", "front_right_wheel_joint"]
		else:
			state.name = [Prefix+"back_right_joint",Prefix+ "back_left_joint",Prefix+"front_left_steer_joint",Prefix+"front_left_wheel_joint",
							Prefix+"front_right_steer_joint", Prefix+"front_right_wheel_joint"]
		
		#print ("mag: ",car.get_magnetometer_data())		
		edition.data = car.get_version()*1.0
		battery.data = car.get_battery_voltage()*1.0
		ax, ay, az = car.get_accelerometer_data()
		gx, gy, gz = car.get_gyroscope_data()
		mx, my, mz = car.get_magnetometer_data()
		mx = mx * 1.0
		my = my * 1.0
		mz = mz * 1.0
		vx, vy, angular = car.get_motion_data()
		'''print("vx: ",vx)
		print("vy: ",vy)
		print("angular: ",angular)'''
		# 发布陀螺仪的数据
		# Pubish gyroscope data
		imu.header.stamp = time_stamp.to_msg()
		imu.header.frame_id = imu_link
		imu.linear_acceleration.x = ax*1.0
		imu.linear_acceleration.y = ay*1.0
		imu.linear_acceleration.z = az*1.0
		imu.angular_velocity.x = gx*1.0
		imu.angular_velocity.y = gy*1.0
		imu.angular_velocity.z = gz*1.0

		mag.header.stamp = time_stamp.to_msg()
		mag.header.frame_id = imu_link
		mag.magnetic_field.x = mx*1.0
		mag.magnetic_field.y = my*1.0
		mag.magnetic_field.z = mz*1.0
		
		# 将小车当前的线速度和角速度发布出去
		# Publish the current linear vel and angular vel of the car
		twist.linear.x = vx *1.0
		twist.linear.y = vy *1.0
		twist.angular.z = angular*1.0    
		velPublisher.publish(twist)
		# print("ax: %.5f, ay: %.5f, az: %.5f" % (ax, ay, az))
		# print("gx: %.5f, gy: %.5f, gz: %.5f" % (gx, gy, gz))
		# print("mx: %.5f, my: %.5f, mz: %.5f" % (mx, my, mz))
		# rospy.loginfo("battery: {}".format(battery))
		# rospy.loginfo("vx: {}, vy: {}, angular: {}".format(twist.linear.x, twist.linear.y, twist.angular.z))
		imuPublisher.publish(imu)
		magPublisher.publish(mag)
		volPublisher.publish(battery)
		EdiPublisher.publish(edition)
		
		
			
int main():
{
	rclpy::init() 
	driver = yahboomcar_driver('driver_node')
	rclpy::spin(driver)
}	
		
