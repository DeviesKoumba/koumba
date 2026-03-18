#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
import canopen
import os

# Import des modules locaux
from .servo_controller import KincoServo
from .calculations import rads_to_kinco_units, kinco_units_to_rads, kinco_pos_to_rad, rps2_to_kinco_accel

def decode_op_mode(mode_int):
    """Traduit le code du mode de fonctionnement en texte."""
    if mode_int is None:
        return "Erreur de lecture"
        
    modes = {
        1: "Profile Position Mode",
        3: "Profile Velocity Mode",      # Mode cible pour le robot
        4: "Profile Torque Mode",
        6: "Homing Mode",
        8: "Cyclic Synchronous Position",
        9: "Cyclic Synchronous Velocity",
        10: "Cyclic Synchronous Torque"
    }
    return modes.get(mode_int, f"Mode inconnu ({mode_int})")

class MotorCanNode(Node):
    def __init__(self):
        super().__init__('motor_can_node')

        # --- PARAMÈTRES ROS ---
        self.declare_parameter('can_interface', 'can0')
        self.interface = self.get_parameter('can_interface').value
        
        self.declare_parameter('left_node_id', 2)
        self.left_node_id = self.get_parameter('left_node_id').value
        
        self.declare_parameter('right_node_id', 3)
        self.right_node_id = self.get_parameter('right_node_id').value
        
        self.declare_parameter('accel_rps2', 100.0)
        self.accel_rps2 = self.get_parameter('accel_rps2').value
        
        self.declare_parameter('eds_file_path', '/home/koumba/robot_ws/src/battery_motor_pkg/config/Kinco_FD_20250417_V1.2.eds')
        eds_path = self.get_parameter('eds_file_path').value

        # --- INITIALISATION MATÉRIELLE ---
        self.network = canopen.Network()
        self.active = False
        
        try:
            if not os.path.exists(eds_path):
                self.get_logger().error(f"Fichier EDS introuvable au chemin : {eds_path}")
                return

            self.network.connect(bustype='socketcan', channel=self.interface)
            self.get_logger().info(f"Connexion au bus CAN sur {self.interface} ")

            # Création des noeuds en passant le fichier EDS !
            self.left_motor = KincoServo(self.network.add_node(self.left_node_id, eds_path))
            self.right_motor = KincoServo(self.network.add_node(self.right_node_id, eds_path))
            
            # Séquence de préparation
            self.get_logger().info("Réinitialisation et configuration du mapping PDO...")
            self.left_motor.fault_reset()
            self.right_motor.fault_reset()
            
            self.left_motor.configure_pdo()
            self.right_motor.configure_pdo()

            accel_dec = rps2_to_kinco_accel(self.accel_rps2)
            self.left_motor.init_velocity_mode(accel=accel_dec)
            self.right_motor.init_velocity_mode(accel=accel_dec)

            # Passage en mode Opérationnel (Active les envois PDO en tâche de fond)
            self.network.nmt.state = 'OPERATIONAL'
            self.active = True
            self.get_logger().info("Robot prêt. Moteurs opérationnels.")

        except Exception as e:
            self.get_logger().error(f"Erreur d'initialisation matérielle : {e}")
            self.active = False

        # INTERFACES ROS2: Publishers and subscribers
        self.sub_cmd = self.create_subscription(Float64MultiArray, '/wheel_cmd', self.wheel_cmd_callback, 10)
        self.pub_states = self.create_publisher(JointState, '/wheel_states', 10)

        self.srv_mode = self.create_service(Trigger, '/get_motors_mode', self.mode_service_callback)# j'ai ajouté cette topics pour la suppervion du mode

        # Boucle de rafraîchissement temps réel (50Hz)
        self.timer = self.create_timer(0.02, self.control_etat_moteur)


    def control_etat_moteur(self): #objet publié par le publisher
        """Lecture du cache local (TxPDO) et publication"""
        if not self.active:
            return

        left_motor_data = self.left_motor.get_all_data()
        rigth_motor_data = self.right_motor.get_all_data()

        if left_motor_data and rigth_motor_data:
            state_msg = JointState()
            state_msg.header.stamp = self.get_clock().now().to_msg()
            state_msg.name = ['left_wheel_joint', 'right_wheel_joint']
            
            state_msg.velocity = [
                kinco_units_to_rads(left_motor_data["velocity"]), 
                kinco_units_to_rads(rigth_motor_data["velocity"])
            ]
            state_msg.position = [
                kinco_pos_to_rad(left_motor_data["position"]), 
                kinco_pos_to_rad(rigth_motor_data["position"])
            ]
            self.pub_states.publish(state_msg) #publication de l'etat des roues sur /wheel_states
            

    def wheel_cmd_callback(self, msg): #callback du subscriber
        """Envoi de la consigne (RxPDO)"""
        if not self.active or len(msg.data) < 2:
            return

        left_units = rads_to_kinco_units(msg.data[0])
        rigth_units = rads_to_kinco_units(msg.data[1])

        self.left_motor.set_target_velocity(left_units) #ecriture de la vitesse sur le moteur gauche
        self.right_motor.set_target_velocity(rigth_units)# ecriture de la vitesse sur le moteur droit


    def mode_service_callback(self, request, response):
        
        self.get_logger().info("Requête SDO reçue : Lecture du Mode de fonctionnement...")
        
        if not self.active:
            response.success = False
            response.message = "Échec : Le bus CAN n'est pas actif."
            return response

        mode_left_motor = self.left_motor.get_operation_mode()
        mode_rigth_motor = self.right_motor.get_operation_mode()

        if mode_left_motor is not None and mode_rigth_motor is not None:
            txt_mode_left_motor = decode_op_mode(mode_left_motor)
            txt_mode_rigth_motor = decode_op_mode(mode_rigth_motor)
            
            response.success = True
            response.message = (
                f"\n--- MOTEUR GAUCHE ---\nMode : [{mode_left_motor}] {txt_mode_left_motor}"
                f"\n\n--- MOTEUR DROIT ---\nMode : [{mode_rigth_motor}] {txt_mode_rigth_motor}"
            )
        else:
            response.success = False
            response.message = "Erreur de communication SDO avec les variateurs (Lecture Mode)."

        return response


    def destroy_node(self):
        """Sécurité à la fermeture du noeud ROS"""
        if self.active:
            self.get_logger().info("Coupure de la puissance des moteurs...")
            try:
                self.left_motor.disable_voltage()
                self.right_motor.disable_voltage()
                self.network.disconnect()
            except:
                pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MotorCanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()