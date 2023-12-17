import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import time
import math
from pySerialTransfer import pySerialTransfer as txfer
import serial

class CircleAnimation:
    def __init__(self):
        self.fig, self.ax = plt.subplots()
        self.link1_length = 40
        self.link2_length = 60
        self.arm1 = Line2D([], [], color='red', linewidth=2)
        self.ax.add_artist(self.arm1)
        self.arm2 = Line2D([], [], color='blue', linewidth=2)
        self.ax.add_artist(self.arm2)

        # Define the serial port and baud rate
        # serial_port = "COM6"  # Replace with your actual serial port
        # baud_rate = 115200

        # # Create a PySerialTransfer object
        # self.link = txfer.SerialTransfer(serial_port, baud_rate)

        # # Open the serial port
        # self.link.open()
        # time.sleep(5)

        # self.ser = serial.Serial('COM6', 9600, timeout=1)
        self.ser = serial.Serial('COM6', 115200, timeout=1)


    def forward_kinematics(self, joint_angles):
        x = self.link1_length * np.cos(joint_angles[0]) + self.link2_length * np.cos(np.sum(joint_angles))
        y = self.link1_length * np.sin(joint_angles[0]) + self.link2_length * np.sin(np.sum(joint_angles))
        return np.array([x, y])

    # def draw_arms(joint_angles):
    def servo_serial(self,servo1_angle,servo2_angle,servo3_angle):
        # Make sure the angles are within a valid range (adjust as needed)
        # servo2_angle=0
        # servo1_angle =90

        servo1_angle = max(0, min(180, servo1_angle))
        servo2_angle = max(0, min(180, 180-servo2_angle))
        servo3_angle = max(0, min(180, servo3_angle))

        command = f"{servo1_angle},{servo2_angle},{servo3_angle}\n"
        self.ser.write(command.encode())
        time.sleep(0.1)  # Add a small delay to ensure the command is received


    def move_servo(self, servo1_angle,servo2_angle,servo3_angle):
        send_size = 0
            
        # Send servo angles
        # servo1_angle = int(input("Enter angle for servo 1: "))
        # servo2_angle = int(input("Enter angle for servo 2: "))
        # servo3_angle = int(input("Enter angle for servo 3: "))
        # print(servo1_angle,servo2_angle,servo3_angle)
        # servo_angles = [servo1_angle,
        #                     servo2_angle,
        #                     servo3_angle]
        
        # Send servo angles list in a single packet
        # send_size += self.link.tx_obj(servo_angles,send_size)
        # self.link.send(send_size)
        print(send_size)
        print(type(servo1_angle))
        servo1_size = self.link.tx_obj(servo1_angle,send_size)
        send_size += servo1_size
        print('send size 1--> ',send_size )
        servo2_size = self.link.tx_obj(servo2_angle,send_size)
        send_size += servo2_size
        print('send size 2--> ',send_size )
        servo3_size = self.link.tx_obj(servo3_angle,send_size)
        send_size += servo3_size
        print('send size 3--> ',send_size )
        
        # # Transmit all the data to send in a single packet
        # self.link.send(send_size)
        
        # Wait for a response and report any errors while receiving packets
        while not self.link.available():
            # print('ho kya rha hai')
            if self.link.status < 0:
                if self.link.status == txfer.CRC_ERROR:
                    print('ERROR: CRC_ERROR')
                elif self.link.status == txfer.PAYLOAD_ERROR:
                    print('ERROR: PAYLOAD_ERROR')
                elif self.link.status == txfer.STOP_BYTE_ERROR:
                    print('ERROR: STOP_BYTE_ERROR')
                else:
                    print('ERROR: {}'.format(self.link.status))

        # # Parse response servo angles list
        # rec_servo_angles = self.link.rx_obj(obj_type=type(servo_angles), obj_byte_size=send_size, list_format='i')
        
        # # Display the received servo angles
        # print('Sent Servo Angles: {}'.format(servo_angles))
        # print('Received Servo Angles: {}'.format(rec_servo_angles))
        # print(' ')
        
        # Parse response servo angles
        # print('rx obj -->>>> ',self.link.rx_obj(obj_type=int))
        rec_servo1_angle = self.link.rx_obj(obj_type=type(servo1_angle), obj_byte_size=servo1_size)
        rec_servo2_angle = self.link.rx_obj(obj_type=type(servo2_angle), obj_byte_size=servo2_size)
        rec_servo3_angle = self.link.rx_obj(obj_type=type(servo3_angle), obj_byte_size=servo3_size)
        
        # Display the received servo angles
        print('Sent Servo Angles: {} {} {}'.format(servo1_angle, servo2_angle, servo3_angle))
        print('Received Servo Angles: {} {} {}'.format(rec_servo1_angle, rec_servo2_angle, rec_servo3_angle))
        print(' ')

        # Wait for a response and report any errors while receiving packets
        # while not self.link.available():
        #     if self.link.status < 0:
        #         if self.link.status == txfer.CRC_ERROR:
        #             print('ERROR: CRC_ERROR')
        #         elif self.link.status == txfer.PAYLOAD_ERROR:
        #             print('ERROR: PAYLOAD_ERROR')
        #         elif self.link.status == txfer.STOP_BYTE_ERROR:
        #             print('ERROR: STOP_BYTE_ERROR')
        #         else:
        #             print('ERROR: {}'.format(self.link.status))

        # # Parse response servo2_angle
        # rec_servo2_angle = self.link.rx_obj(obj_type=int, obj_byte_size=4, list_format='i')
        # print('Sent Servo Angle 2: {} | Received Servo Angle 2: {}'.format(servo2_angle, rec_servo2_angle))

        # # Send servo3_angle
        # send_size = 0
        # send_size += self.link.tx_obj(servo3_angle, val_type=txfer.DTYPE_INT, obj_byte_size=4)
        # self.link.send(send_size)

        # # Wait for a response and report any errors while receiving packets
        # while not self.link.available():
        #     if self.link.status < 0:
        #         if self.link.status == txfer.CRC_ERROR:
        #             print('ERROR: CRC_ERROR')
        #         elif self.link.status == txfer.PAYLOAD_ERROR:
        #             print('ERROR: PAYLOAD_ERROR')
        #         elif self.link.status == txfer.STOP_BYTE_ERROR:
        #             print('ERROR: STOP_BYTE_ERROR')
        #         else:
        #             print('ERROR: {}'.format(self.link.status))

        # # Parse response servo3_angle
        # rec_servo3_angle = self.link.rx_obj(obj_type=int, obj_byte_size=4, list_format='i')
        # print('Sent Servo Angle 3: {} | Received Servo Angle 3: {}'.format(servo3_angle, rec_servo3_angle))


    def inverse_kinematics(self, target_position):
        x, y = target_position
        D = (x**2 + y**2 - self.link1_length**2 - self.link2_length**2) / (2 * self.link1_length * self.link2_length)
        theta2 = np.arctan2(-np.sqrt(1 - D**2), D)
        theta1 = np.arctan2(y, x) - np.arctan2(self.link2_length * np.sin(theta2), self.link1_length + self.link2_length * np.cos(theta2))
        dist = 100000
        for offset in range(50):
            theta2_check = np.arctan2(-np.sqrt(1 - D**2), D) + (offset*(3.14/50))
            # print('D --> ', D)
            # print('theta 2 --->  ',theta2)
            theta1_check = np.arctan2(y, x) - np.arctan2(self.link2_length * np.sin(theta2_check), self.link1_length + self.link2_length * np.cos(theta2_check))
            # print('theta 1 --->  ',theta1)
            reached = self.forward_kinematics([theta1_check,theta2_check])
            dist_check = math.sqrt((x-reached[0])**2 + (y-reached[1])**2)
            # print(dist_check)
            if dist_check < dist:
                dist = dist_check
                theta1 = theta1_check
                theta2 = theta2_check
        print(theta1,theta2+3.14)
        return np.array([theta1, theta2])

    def check_reachability(self):
        # Define the grid
        x_range = np.linspace(-100, 100, 100)
        y_range = np.linspace(0, 100, 50)

        # Create a meshgrid from x and y ranges
        X, Y = np.meshgrid(x_range, y_range)

        # Initialize arrays to store reachable and unreachable points
        reachable_points = np.zeros_like(X, dtype=bool)

        # Calculate inverse kinematics for each point in the grid
        for i in range(len(x_range)):
            for j in range(len(y_range)):
                x = X[j, i]
                y = Y[j, i]
                theta1, theta2 = self.inverse_kinematics([x, y])

                # Check if both angles are less than 3.14
                reachable_points[j, i] = (theta1 < 3.14) and (theta2 + 3.14 < 3.14)

        # Plot the reachable and unreachable regions
        plt.figure(figsize=(8, 6))
        plt.title('Reachable and Unreachable Regions')
        plt.contourf(X, Y, reachable_points, levels=[0, 0.5, 1], colors=['red', 'green'], alpha=0.3)
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.colorbar()

        # Show the plot
        plt.show()


    def update_plot(self, i):
        ## keyboard input
        target =i

        ### straight line
        # if i <30:
        # target =[i-30,60]
        # else:
        #     target = [0,60]


        #### SQUARE >>>>>>>>>>>>>>

        # if i<25:
        #     target = [10+i, 50]
        # if i>=25 and i<50:
        #     target = [35,50-(i-25)]
        # if i>=50 and i<75:
        #     target = [35 -(i-50),25]
        # if i>=75 :
        #     target = [10,25+(i-75)]

        # if i<25:
        #     target = [-20+(i*2), 70]
        # if i>=25 and i<50:
        #     target = [30,70-((2*i)-50)]
        # if i>=50 and i<75:
        #     target = [30 -((i*2)-100),20]
        # if i>=75 :
        #     target = [-20,20+((i*2)-150)]
        # if i<10:
        #     target = [-10+(i*2), 50]
        # if i>=10 and i<20:
        #     target = [35,50-((i*2)-20)]
        # if i>=20 and i<30:
        #     target = [35 -(i-50),25]
        # if i>=30 :
        #     target = [10,25+(i-75)]
        self.ax.cla()
        circle1 = plt.Circle((0, 0), 1, color='blue')
        circle2 = plt.Circle((target[0], -target[1]), 5, color='green')
        self.ax.add_artist(circle1)
        self.ax.add_artist(circle2)
        # joint_angles = [(3.14/25)*i , (3.14/25)*i ]
        
        joint_angles = self.inverse_kinematics(target)
        if joint_angles[0] or (joint_angles[1]+3.14) > 3.14:
            print('angles limit ')
        print(joint_angles[0]*57,(joint_angles[1]+3.14)*57)

        arm1_x = [0 , self.link1_length * np.cos(joint_angles[0])]
        arm1_y = [0,-(self.link1_length * np.sin(joint_angles[0]))]
        arm2_x = [self.link1_length * np.cos(joint_angles[0]), self.link1_length * np.cos(joint_angles[0]) + self.link2_length * np.cos(np.sum(joint_angles)) ]
        arm2_y =  [-(self.link1_length * np.sin(joint_angles[0])), -(self.link1_length * np.sin(joint_angles[0]) + self.link2_length * np.sin(np.sum(joint_angles)) )]
        pitch_angle = 180-((joint_angles[0]-(joint_angles[1]))*57)
        pitch_angle = np.arctan2(arm2_y[1]-arm2_y[0], arm2_x[1]-arm2_x[0]) + 3.14
        # if pitch_angle >180:
        #     pitch_angle = 360 -pitch_angle
        print('pitch angle --> ', pitch_angle*57)
        # if i%3 ==0:
        #     # self.move_servo(int(joint_angles[0]*57),int(joint_angles[1]*57)+180,int(0))
        
        
        self.servo_serial(int(joint_angles[0]*57),int(joint_angles[1]*57)+180,pitch_angle*57 - 10)
        # self.servo_serial(target[0],target[1],pitch_angle*57 - 10)
        
        circle3 = plt.Circle((arm2_x[1], arm2_y[1]), 3, color='red')
        self.ax.add_artist(circle3)
        self.arm1.set_data(arm1_x, arm1_y)
        self.arm2.set_data(arm2_x, arm2_y)

        self.ax.add_artist(self.arm1)
        self.ax.add_artist(self.arm2)

        self.ax.set_xlim(-self.link1_length - self.link2_length, self.link1_length + self.link2_length)
        self.ax.set_ylim(-self.link1_length - self.link2_length, self.link1_length + self.link2_length)

    def run_animation(self):
        # for i in range(100):
        #     self.update_plot(i)
        #     plt.pause(0.001)
        #     print(i)
        #     time.sleep(0.1)
        trajectory = [[0,90],[0,80],[0,70],[0,60],[0,50],[10,40],[20,35],[40,25],[60,20],[70,30],[70,40],[70,50]]
        # while True:
        #     x = int(input('enter x :'))
        #     y = int(input('enter y :'))
        #     self.update_plot([x,y])
        #     plt.pause(0.001)
        # #     print(i)
        #     time.sleep(0.1)
        for _ in range(5):
            l = len(trajectory)
            for i in range(l):
                print(i)
                self.update_plot(trajectory[i])
                plt.pause(0.001)
                time.sleep(0.4)
            for j in range(l):
                print(j)
                self.update_plot(trajectory[l-1-j])
                plt.pause(0.001)
                time.sleep(0.4)

# Example usage:
animation = CircleAnimation()
animation.run_animation()
# animation.check_reachability()
plt.show()
