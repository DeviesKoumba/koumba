import math

# --- CONFIGURATION MATÉRIELLE ---
GEAR_RATIO = 30.0    # Réducteur 30:1
ENCODER_RES = 10000  # Résolution de l'encodeur Kinco MD

def rads_to_kinco_units(rad_s):
    """Convertit les Rad/s (ROS) en unités DEC internes de Kinco."""
    wheel_rpm = (rad_s * 60.0) / (2.0 * math.pi)
    motor_rpm = wheel_rpm * GEAR_RATIO
    kinco_dec = (motor_rpm * 512 * ENCODER_RES) / 1875
    return int(kinco_dec)

def kinco_units_to_rads(kinco_dec):
    """Convertit les unités DEC Kinco en Rad/s pour ROS."""
    motor_rpm = (kinco_dec * 1875) / (512 * ENCODER_RES)
    wheel_rpm = motor_rpm / GEAR_RATIO
    return (wheel_rpm * 2.0 * math.pi) / 60.0

def kinco_pos_to_rad(kinco_counts):
    """Convertit les counts de l'encodeur en radians (odométrie)."""
    motor_rad = (kinco_counts * 2.0 * math.pi) / ENCODER_RES
    return motor_rad / GEAR_RATIO

def rps2_to_kinco_accel(rps2):
    """
    Convertit l'accélération (rps/s) en unités internes DEC Kinco.
    Formule Kinco (0x6083 / 0x6084) : DEC = [(rps/s * 65536 * Encoder_Resolution) / 4000000]
    """
    dec = (rps2 * 65536.0 * ENCODER_RES) / 4000000.0
    return int(dec)