#!/usr/bin/env python3
import rclpy
import serial
from rclpy.node import Node
from covaps.msg import Order
#import RPi.GPIO as GPIO
from time import sleep

SPEED_LIMIT = 500




class ComNode(Node):
    def __init__(self):
        super().__init__("com_node")
        self.get_logger().info("com node started")
        self.pathMicro = "/dev/ttyUSB0"
        self.baudrate = 115200
        self.arduino = serial.Serial(self.pathMicro, self.baudrate, timeout= 1)

        self.stm = serial.Serial()
        self.stm.port = self.pathMicro
        self.stm.baudrate = self.baudrate

        self.cmd_recv = self.create_subscription(Order, "/monthlery/cmd_car", self.rcv_order, 10)

    def rcv_order(self, order: Order):
        if (order.type == "speed"):
            self.changeSpeed(order.val)
        elif (order.type == "angular"):
            self.changeAngular(order.val)
        else :
            self.get_logger().warning("invalide type")


    def changeSpeed(self,  speedValue):
        self.get_logger().info("change speed " + str(speedValue))
        self.changePWMSpeed(speedValue)

    def changeAngular(self, angularPos):
        self.get_logger().info("change angular position " + str(angularPos))
        self.changePWMDir(angularPos)

    
    def changePWMDir(self, angleValue):
        order_type = 97
        arg = angleValue
        self.sendOrder(order_type, arg)
        


    def changePWMSpeed(self, speedValue):
        order_type = 98
        arg = speedValue
        if (arg <= SPEED_LIMIT) :
            self.sendOrder(order_type, arg)


    def sendOrder(self, octet1 : int, octet2 : int): #octet1 and octet2 should not be more that 8 bits

        trame = int.to_bytes( (octet1 << 8 ) | octet2, 2, "big")
        self.arduino.write(trame)




def main(args=None):
    rclpy.init(args=args)
    node = ComNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()