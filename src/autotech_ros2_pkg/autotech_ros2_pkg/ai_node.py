##
#%%
import numpy
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from math import sqrt, atan, pi

from covaps.msg import Order

class AI(Node):
    def __init__(self,**kargs):
        super().__init__("ai_node")

        self.laser_scan = Odometry()
        self.occupancy = OccupancyGrid()
        self.odometry = Odometry()
        self.cmd_car = self.create_publisher(Order, "/monthlery/cmd_car", 10)
        ## PUBLISHER
        self.cmd_vel_publisher = self.create_publisher(
            Twist, "ai_node/topic/twist", 10
        )

        ## SUBSCRIBER
        self.laser_scan_subscriber = self.create_subscription(
            Odometry, "localisation/topic/odometry",self.callback_laser_scan,10
        )
        self.occupancy_grid_subscriber = self.create_subscription(
            OccupancyGrid, "map_server/topic/occupancy_grid",self.callback_occupancy,10
        )
        self.odometry_subscriber = self.create_subscription(
            Odometry, "odometry/topic/odometry",self.callback_odometry,10
        )
        self.get_logger().info("AI abstract node has been started")

    def callback_laser_scan(self,odometry : Odometry):
        self.laser_scan = odometry

    def callback_occupancy(self,occupancy : OccupancyGrid):
        self.occupancy = occupancy

    def callback_odometry(self,odometry : Odometry):
        self.odometry = odometry

    #abstractmethod
    def subToObs(self) -> list: ##A COMPLETER
        """Take the information in the topics in wich this node is subscribes
         and adapt its for the AI input """
        pass

    #abstractmethod
    def outToPublish(self,output : list) -> Twist: ## A COMPLETER
        """Take the output of the AI 
        and publish it in an appropriate format for the topic (Twist)"""
        obs = self.subToObs(output)
        action, _ = model.predict(obs, deterministic = True)
        Sx, Sy = numpy.float64(action[0]), numpy.float64(action[1]) #Speed on the x and y axes
        linear_speed = sqrt(Sx**2 + Sy**2)
        if Sx == 0:
            if Sy > 0:
                angular_speed = pi/2
            elif Sy < 0:
                angular_speed = -pi/2
            else: 
                angular_speed = 0
        else:
            angular_speed = atan(Sy/Sx)
        if Sx < 0:
            linear_speed = - linear_speed
        order_linear = Order()
        order_linear.val = linear_speed
        order_linera.type = "speed"
        self.cmd_car.publish(order_linear)
        order_angular = Order()
        order_angular.val = angular_speed
        order_angular.type = "angular"
        self.cmd_car.publish(order_angular)
        self.get_logger().info("Model predict: {}".format(str(action))
    
    
    def publish(self):
        self.cmd_vel_publisher(
            self.OutToPublish(
                self.ai(
                    self.subToInput()
                )
            )
        )        


##
#%%
from std_msgs.msg import Float64MultiArray
from stable_baselines3 import PPO
"""
            "observation": {
                "type": "LidarObservation", #LidarObservation Kinematics
                "maximum_range": [50],
                "as_image": False,
                "align_to_vehicle_axes": True,
                "cells": 17,
                
            },
            "action": {
                "type": "ContinuousAction",#classe qui gère les mvts dans le fichier highway-env-master\highway_env\envs\common\action.py
                "longitudinal":True,
                "lateral": True,
                "speed_range": [5, 10]
            },
            """ 
models_path = "~/ros2_ws/src/monthlery/modele/Modèles finaux/"
model = PPO.load(models_path+"bien2/model")#mettre le nom du dossier qui contient le modèle
class AINodeTest(AI):
    def __init__(self):
        super().__init__()
        # Subscriber

        self.obs_subscriber = self.create_subscription(
            Float64MultiArray, "obs/topic/array", self.callback_pub,10
        )
        self.get_logger().info("AI test node has been started")

    def subToObs(self, array : Float64MultiArray):
        return [ [e] for e in array.data]
"""
    def callback_obs(self, array : Float64MultiArray):
        obs = self.subToObs(array)
        self.get_logger().info(f"Observation received : {str(obs)}")
        action, _states = model.predict(obs, deterministic=True)
        self.get_logger().info("Model predict : {}".format(str(action)) )
        cmd_vel = self.actionToTwist(action)
        self.cmd_vel_publisher.publish(cmd_vel)"""
    
    def callback_pub(self, array : Float64MultiArray):
        obs = self.subToObs(array)
        action, _ = self.model.predict(obs, deterministic=True)
        Sx, Sy = numpy.float64(action[0]), numpy.float64(action[1])
        linear_speed, angular_speed = get_lin_and_ang_speed(Sx, Sy)
        order_angular = create_order("angular", angular_speed)
        self.cmd_car.publish(order_angular)
        order_linear = create_order("speed", linear_speed)
        self.cmd_car.publish(order_linear)
        self.get_logger.publish(order_linear)
"""
    #abstractmethod
    def actionToTwist(self,action) -> Twist: ## A COMPLETER
        """Take the output of the model, an action and return the command velocity in Twist type"""
        cmd_vel = Twist()
        cmd_vel.angular.x = numpy.float64(action[0])
        cmd_vel.angular.y = numpy.float64(action[1])
        return cmd_vel
   """     
        
def create_order(typ, val):
    order = Order()
    order.val = val
    order.type = typ
    return order

def get_lin_and_ang_speed(Sx, Sy):
    linear_speed = sqrt(Sx**2 + Sy**2)
    if Sx ==0:
        if Sy > 0:
            angular_speed = pi/2
        elif Sy < 0:
            angular_speed = -pi/2
        else:
            angular_speed = 0
    else:
        angular_speed = atan(Sy/Sx)
    if Sx < 0:
        linear_speed = -linear_speed
    return linear_speed, angular_speed


def main(args=None):
    models_path = "~/ros2_ws/src/monthlery/modele/Modèles finaux/"
    model = PPO.load(models_path+"bien2/model")    #mettre le nom du dossier qui contient le modèle

    rclpy.init(args=args)
    node = AINodeTest()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
