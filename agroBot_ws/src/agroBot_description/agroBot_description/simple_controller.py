import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

from std_msgs.msg import Float64MultiArray, Int8, Int64MultiArray
import numpy as np
import math


from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import numpy as np
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
from rclpy.time import Time
from rclpy.constants import S_TO_NS


from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
import tf2_ros
import numpy as np


class FourWheeledSteeringController(Node):
    def __init__(self):
        super().__init__('four_wheeled_steering_controller')
        
        # Tópico de publicação para os comandos de velocidade, esterçamento e posição base
        self.control_pub = self.create_publisher(Float64MultiArray, 'simple_velocity_controller/commands', 10)
        
        # Tópico de subscrição para o comando de velocidade linear e angular
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_control_callback, 10)
        self.subscription  # prevent unused variable warning

    def cmd_control_callback(self, msg):
        # Receber os comandos de velocidade linear e angular
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z

        # Implements the differential kinematic model
        # Given v_x, v_y and w, calculate the velocities of the wheels
        robot_speed = np.array([[linear_velocity],
                                [angular_velocity]])
        a = 0.300
        l = 0.618
        d = 0.560
        self.jacobiano_inverso = np.array([[1/a, -d/a],
                                           [1/a, -d/a],
                                           [1/a, d/a],
                                           [1/a, d/a]])

        #extrai o valor de velocidade para cada roda no eixo x e eixo y
        wheel_speed = np.matmul(self.jacobiano_inverso, robot_speed) 
        
        ###Calcula velocidade das rodas para direção
        omega1 = float(wheel_speed[0])
        omega2 = float(wheel_speed[1])
        omega3 = float(wheel_speed[2])
        omega4 = float(wheel_speed[3])
        
       
        self.get_logger().info("omega1 %s" % omega1)
        self.get_logger().info("omega2 %s" % omega2)
        self.get_logger().info("omega3 %s" % omega3)
        self.get_logger().info("omega4 %s" % omega4)

        # Criar mensagem Float64MultiArray para os comandos de velocidade
        control_msg = Float64MultiArray()

        # Se o comando de velocidade for negativo, então inverte a referência de velocidades das rodas de tração
        if msg.linear.x > 0:
            control_msg.data = [omega1, omega2, omega3, omega4]
            # Publicar os comandos de velocidade
            self.control_pub.publish(control_msg)

        else:
            control_msg.data = [omega1, omega2, omega3, omega4]
            # Publicar os comandos de velocidade
            self.control_pub.publish(control_msg)
  

def main(args=None):
    rclpy.init(args=args)
    node = FourWheeledSteeringController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
