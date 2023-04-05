##
#%%
import numpy
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from math import sqrt, atan, pi


from std_msgs.msg import Float64MultiArray
from stable_baselines3 import PPO

from covaps.msg import Order

models_path = "~/ros2_ws/src/monthlery/modele/Modèles finaux/"

model_name = "bien2/model"

class AI(Node):
    number_laser_points = 1081
    model = PPO.load(models_path + model_name)#mettre le nom du dossier qui contient le modèle

    def __init__(self,**kargs):
        super().__init__("ai_node")

        self.laser_scan = Odometry()
        self.occupancy = OccupancyGrid()
        self.odometry = Odometry()

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


##
#%%

class AINode(AI):
    def __init__(self, **kargs):
        super().__init__(**kargs)

        # Publisher
        self.cmd_car = self.create_publisher(Order, "/monthlery/cmd_car", 10)
        # End initialize
        self.get_logger().info("AI node has been started")


    def callback_pub(self, array : Float64MultiArray):
        
        action, _ = self.model.predict(
            self.angle_opening(array.data, 45),
            deterministic=True
        )
        Sx, Sy = numpy.float64(action[0]), numpy.float64(action[1])

        linear_speed, angular_speed = get_lin_and_ang_speed(Sx, Sy)

        order_angular = create_order("angular", angular_speed)
        self.cmd_car.publish(order_angular)
        order_linear = create_order("speed", linear_speed)
        self.cmd_car.publish(order_linear)
        
        self.get_logger().info("Model predict: {}".format(str(action)))


    def angle_opening(self, obs, theta, number_points=17):
        ''' Return the point that are in the opening angle in the direction of the car
        PARAMETERS
        ----------
            theta : float
                Angle in degree
        '''
        angle_by_point = 360 / self.number_laser_points
        opening_index_0 = int((180 - theta/2) / angle_by_point)
        opening_index_1 = int((180 + theta/2) / angle_by_point)
        step = (opening_index_1 - opening_index_0)//number_points
        return obs[opening_index_0:opening_index_1:step]


##
#%%


        
def create_order(type_, val_):
    order = Order()
    order.val = val_
    order.type = type_
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

    rclpy.init(args=args)
    node = AINode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
