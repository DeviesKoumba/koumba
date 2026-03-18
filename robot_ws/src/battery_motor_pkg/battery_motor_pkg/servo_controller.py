import time

class KincoServo:
    def __init__(self, node):
        self.node = node

    def fault_reset(self):
        """Réinitialise les erreurs du variateur."""
        self.node.sdo['Controlword'].raw = 0x0080
        time.sleep(0.1)

    def disable_voltage(self): # la coupure se fera via le relais de secu.
        """Coupe la puissance."""
        self.node.sdo['Controlword'].raw = 0x0000

    def configure_pdo(self):
        """
        Configuration du mapping PDO grâce à l'EDS."""
        # --- RxPDO1 (Reçoit du PC : Controlword + Vitesse Cible) ---
        self.node.pdo.rx[1].clear()
        self.node.pdo.rx[1].add_variable('Controlword')
        self.node.pdo.rx[1].add_variable('Target_velocity')
        self.node.pdo.rx[1].enabled = True
        self.node.pdo.rx[1].trans_type = 255
        self.node.pdo.rx[1].save()

        # --- TxPDO1 (Envoie au PC : Vitesse Réelle + Code d'Erreur) ---
        self.node.pdo.tx[1].clear()
        self.node.pdo.tx[1].add_variable('speed_real')
        self.node.pdo.tx[1].add_variable('error code for DS301') 
        self.node.pdo.tx[1].enabled = True
        self.node.pdo.tx[1].trans_type = 254
        self.node.pdo.tx[1].event_timer = 20 
        self.node.pdo.tx[1].save()

        # --- TxPDO2 (Envoie au PC : Position Encodeur + Couple) ---
        self.node.pdo.tx[2].clear()
        self.node.pdo.tx[2].add_variable('Position_actual_value_')
        self.node.pdo.tx[2].add_variable('Actual_Torque')
        self.node.pdo.tx[2].enabled = True
        self.node.pdo.tx[2].trans_type = 254
        self.node.pdo.tx[2].event_timer = 20
        self.node.pdo.tx[2].save()

    def init_velocity_mode(self, accel_dec):
        """Séquence d'allumage initiale via SDO."""
        self.node.sdo['Modes_of_operation'].raw = 3
        self.node.sdo['Profile_acceleration'].raw = int(accel_dec)
        self.node.sdo['Profile_deceleration'].raw = int(accel_dec)

        for cmd in [0x0006, 0x0007, 0x000F]: # Séquence : Shutdown -> Switch On -> Enable
            self.node.sdo['Controlword'].raw = cmd
            time.sleep(0.05)

    
    # lecture du mode du moteur 
  
    def get_operation_mode(self):
        """Lecture du mode des moteurs """
        try:
            return self.node.sdo['Operation_Mode_Buff'].raw
        except Exception:
            return None

    
    # BLOC DE COMMUNICATION TEMPS REELLE AVEC LES MOTEURS
   

    def set_target_velocity(self, units):
        """Envoi de la consigne Broadcast pour envoyer la vitesse cible."""
        try:
            self.node.pdo.rx[1]['Controlword'].raw = 0x000F #pour rendre le moteur pret a enab
            self.node.pdo.rx[1]['Target_velocity'].raw = int(units)
            self.node.pdo.rx[1].transmit() # Expédie instantanément sur le bus
        except Exception:
            pass

    def get_all_data(self):
        """Lecture Instantanée depuis le driver du moteur."""
        try:
            return {
                "velocity": self.node.pdo.tx[1]['speed_real'].phys,
                "error":    self.node.pdo.tx[1]['error code for DS301'].phys,
                "position": self.node.pdo.tx[2]['Position_actual_value_'].phys, 
                "torque":   self.node.pdo.tx[2]['Actual_Torque'].phys
                # L'attribut .phys retourne la donnée au format "entier" lisible

            }
        except Exception:
            # Sécurité au démarrage si le premier message n'est pas encore arrivé
            return None