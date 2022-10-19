#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry

from math import pi, cos, sin

from geometry_msgs.msg import Quaternion

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0; aj /= 2.0; ak /= 2.0
    ci = cos(ai); si = sin(ai)
    cj = cos(aj); sj = sin(aj)
    ck = cos(ak); sk = sin(ak)
    
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    return [cj*sc - sj*cs, cj*ss + sj*cc, cj*cs - sj*sc, cj*cc + sj*ss]

# Create a Quaternion message representiong yaw rotation
def quaternion_msg_from_yaw(yaw:float):
    """Crée un Quaternion à partir d'un angle de lacet (cap)
    
    Fonction utilitaire qui crée un message Quaternion correspondant à une rotation
    représentant l'orientation 'yaw' (cap du robot en radians)

    :param yaw: angle de cap en radians
    """
    q = quaternion_from_euler(0.0, 0.0, yaw)
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])


class WheelOdometryNode(Node):
    def __init__(self):
        """Constructeur de la classe WheelOdometryNode"""

        super().__init__('odometry')
        # Publisher ROS, pour publier le resultat d'odométrie sur le topic 'odometry'
        self._odom_pub = self.create_publisher(Odometry, 'odometry', 10)
        
        # Subscriber : s'abonne au topic 'nxt/encoders' (positions des roues). 
        # La méthode encoders_callback sera appelée à chaque message reçu
        self._encoder_sub = self.create_subscription(JointState, "nxt/encoders", self.encoders_callback, 10)

        # Position et orientation initiales dans le repère odométrique (0,0,0)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Paramètres géométriques du robot
        # TODO: Ajuster les paramètres géométriques / les remplacer par des paramètres ROS2
        self.r = 0.0176  # Rayon d'une roue
        self.L = 0.18    # Voie (distance entre deux roues d'un même essieu)

        # Mémorisation de l'ancienne position de chaque roue
        self._old_left_pos = None
        self._old_right_pos = None

    def encoders_callback(self, motors_state):
        """Callback pour traiter les informations de position des roues (codeurs)"""
        # Extrait les positions angulaires des roues depuis le message ROS reçu
        left_pos = motors_state.position[0]
        right_pos = motors_state.position[1]

        # TODO: Il n'est pas possible de calculer une différence entre deux positions la première fois
        if self._old_left_pos is None or self._old_right_pos is None:
            
            self._old_left_pos=left_pos
            self._old_right_pos=right_pos

        # TODO: Calcul du déplacement angulaire de chaque roue 
        # et mise à jour des positions mémorisées
        d_left  = left_pos-self._old_left_pos
        d_right = right_pos-self._old_right_pos
        self._old_left_pos=left_pos
        self._old_right_pos=right_pos

        # TODO: Calcul du déplacement du robot
        ds     = self.r * (d_left + d_right) / 2
        dtheta = self.r * (d_left - d_right)/ self.L

        # TODO: Mise à jour de la position estimée du robot
        self.x     = self.x  + ds * cos(self.theta)
        self.y     = self.y  + ds * sin(self.theta)
        self.theta = self.theta + dtheta

        self.get_logger().info(f"x={self.x:.3f} y={self.y:.3f} theta={self.theta:.3f}")
        
        # TODO: Crée et publie le message d'odométrie
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.pose.pose.position.x  = self.x
        msg.pose.pose.position.y  = self.y
        msg.pose.pose.orientation = quaternion_msg_from_yaw(self.theta)
        self._odom_pub.publish(msg)
    

def main(args=None):
    rclpy.init(args=args)
    node = WheelOdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
