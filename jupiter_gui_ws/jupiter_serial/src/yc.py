#!/usr/bin/env python
import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from std_msgs.msg import Float64

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Twist!
1 2 3 4 : wheel
5 6 7 8 : joint


t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
hiahiahia
CTRL-C to quit
"""




moveBindings = {
		'1':1,
		'2':2,
		'3':3,
		'4':4,
		'5':5,
		'6':6,
		'7':7,
		'8':8,
		'O':(1,-1,0,0),
		'I':(1,0,0,0),
		'J':(0,1,0,0),
		'L':(0,-1,0,0),
		'U':(1,1,0,0),
		'<':(-1,0,0,0),
		'>':(-1,-1,0,0),
		'M':(-1,1,0,0),
		't':(0,0,1,0),
		'b':(0,0,-1,0),




	       }

a = {
	'wheel1': rospy.Publisher('cmd_vela', Float64, queue_size = 1),
	'wheel2': rospy.Publisher('cmd_velb', Float64, queue_size = 1),
	'wheel3': rospy.Publisher('cmd_velc', Float64, queue_size = 1),
	'wheel4': rospy.Publisher('cmd_veld', Float64, queue_size = 1),
}




speedBindings={
		'q':(1.1,1.1),
		'z':(.9,.9),
		'w':(1.1,1),
		'x':(.9,1),
		'e':(1,1.1),
		'c':(1,.9),
	      }

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

speed = .5
turn = 1

def vels(speed,turn):
	return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    	settings = termios.tcgetattr(sys.stdin)
	
	pub_joint1 = rospy.Publisher('joint1_position_controller/command', Float64, queue_size = 1)
	pub_joint2 = rospy.Publisher('joint2_position_controller/command', Float64, queue_size = 1)
	pub_joint3 = rospy.Publisher('joint3_position_controller/command', Float64, queue_size = 1)
	pub_joint4 = rospy.Publisher('joint4_position_controller/command', Float64, queue_size = 1)

	pub_wheel1 = rospy.Publisher('wheel1_position_controller/command', Float64, queue_size = 1)
	pub_wheel2 = rospy.Publisher('wheel2_position_controller/command', Float64, queue_size = 1)
	pub_wheel3 = rospy.Publisher('wheel3_position_controller/command', Float64, queue_size = 1)
	pub_wheel4 = rospy.Publisher('wheel4_position_controller/command', Float64, queue_size = 1)

	rospy.init_node('teleop_twist_keyboard')


	status = 0
	speed = 0.5
 	turn = 1
	wheel1_twist = Float64()
	wheel2_twist = Float64()
	wheel3_twist = Float64()
	wheel4_twist = Float64()
	joint1_twist = Float64()
	joint2_twist = Float64()
	joint3_twist = Float64()
	joint4_twist = Float64()
	wheel1_twist = 0
	wheel2_twist = 0
	wheel3_twist = 0
	wheel4_twist = 0
	joint1_twist = 0
	joint2_twist = 0
	joint3_twist = 0
	joint4_twist = 0

	try:
		print msg
		print vels(speed,turn)

		while(1):
			key = getKey()
			if key in moveBindings.keys():
				
				if (key == '1'):
					wheel1_twist = 1
				elif (key == '2'):
					wheel2_twist = 1
				elif (key == '3'):
					wheel3_twist = 1
				elif (key == '4'):
					wheel4_twist = 1
				elif (key == '5'):
					joint1_twist = 1
				elif (key == '6'):
					joint1_twist = 1
				elif (key == '7'):
					joint1_twist = 1
				elif (key == '8'):
					joint1_twist = 1

			elif key in speedBindings.keys():
				speed = speed * speedBindings[key][0]
				turn = turn * speedBindings[key][1]

				wheel1_twist = 0
				wheel2_twist = 0
				wheel3_twist = 0
				wheel4_twist = 0
				joint1_twist = 0
				joint2_twist = 0
				joint3_twist = 0
				joint4_twist = 0

				print vels(speed,turn)
				if (status == 14):
					print msg
				status = (status + 1) % 15
			else:
				wheel1_twist = 0
				wheel2_twist = 0
				wheel3_twist = 0
				wheel4_twist = 0
				joint1_twist = 0
				joint2_twist = 0
				joint3_twist = 0
				joint4_twist = 0

				if (key == '\x03'):
					break

			pub_wheel1.publish(wheel1_twist * speed)
			pub_wheel2.publish(wheel2_twist * speed)
			pub_wheel3.publish(wheel3_twist * speed)
			pub_wheel4.publish(wheel4_twist * speed)
			pub_joint1.publish(joint1_twist * turn)
			pub_joint2.publish(joint2_twist * turn)
			pub_joint3.publish(joint3_twist * turn)
			pub_joint4.publish(joint4_twist * turn)

			


			#twist = Float64()
			#twist = 1.448448484848484
			#pub_wheel1.publish(twist)
			
			#a[1].publish(twsit)
			#print "aa"

	except:
		print e

	finally:
		twist = Float64()
		twist = 0
		pub_wheel1.publish(twist)

    		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

