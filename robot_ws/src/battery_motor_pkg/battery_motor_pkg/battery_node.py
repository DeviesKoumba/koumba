#!/usr/bin/env python3
"""
Nœud ROS2 de gestion de batterie (BMS).

Rôle : interroger le BMS toutes les 500 ms via le bus CAN,
publier l'état de la batterie, et couper les moteurs si une
limite critique est franchie.

Deux responsabilités séparées :
  - Supervision  : alertes préventives (logs uniquement)
  - Contrôle     : coupures absolues (E-STOP)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String, Bool
import can
import struct


# ---------------------------------------------------------------------------
# Bits de protection matérielle (registre 0x102, BYTE4~5)
# Chaque bit indique une protection active sur le BMS.
# Source : datasheet §3 Note 1
# ---------------------------------------------------------------------------
DESCRIPTIONS_PROTECTIONS = {
    0:  "Surtension d'une cellule",
    1:  "Sous-tension d'une cellule",
    2:  "Surtension du pack entier",
    3:  "Sous-tension du pack entier",
    4:  "Surchauffe en charge",
    5:  "Froid excessif en charge",
    6:  "Surchauffe en décharge",
    7:  "Froid excessif en décharge",
    8:  "Surcourant en charge",
    9:  "Surcourant en décharge",
    10: "Court-circuit détecté",
    11: "Erreur IC de mesure (front-end)",
    12: "MOSFETs verrouillés par software",
}


class NoeudGestionBatterie(Node):
    """
    Nœud ROS2 principal de gestion de la batterie.

    Publie sur :
      /battery_state               → état complet de la batterie (BatteryState)
      /robot_core/power_management → commande logique (String)
      /hardware/emergency_stop     → arrêt d'urgence (Bool)
    """

    def __init__(self):
        super().__init__('noeud_gestion_batterie')

        # -------------------------------------------------------------------
        # Déclaration puis lecture immédiate des paramètres.
        # Ces seuils correspondent aux limites physiques et chimiques de la
        # batterie LiFePO4 — ils sont fixes par nature.
        # On les déclare (bonne pratique ROS2 : permet de les passer en YAML
        # ou en ligne de commande) puis on les lit une seule fois ici.
        # -------------------------------------------------------------------

        # Seuils de tension (Volts)
        self.declare_parameter('tension_coupure_basse',   44.8)
        self.declare_parameter('tension_alerte_basse',    47.0)
        self.declare_parameter('tension_charge_maximale', 58.4)

        self.tension_coupure_basse   = self.get_parameter('tension_coupure_basse').value
        self.tension_alerte_basse    = self.get_parameter('tension_alerte_basse').value
        self.tension_charge_maximale = self.get_parameter('tension_charge_maximale').value

        # Seuils de courant (Ampères / secondes)
        self.declare_parameter('courant_pic_maximal', 100.0)
        self.declare_parameter('duree_pic_maximale',   10.0)

        self.courant_pic_maximal = self.get_parameter('courant_pic_maximal').value
        self.duree_pic_maximale  = self.get_parameter('duree_pic_maximale').value

        # Plages de température absolues (°C)
        self.declare_parameter('temperature_charge_min',    0.0)
        self.declare_parameter('temperature_charge_max',   45.0)
        self.declare_parameter('temperature_decharge_min', -20.0)
        self.declare_parameter('temperature_decharge_max',  60.0)

        self.temperature_charge_min   = self.get_parameter('temperature_charge_min').value
        self.temperature_charge_max   = self.get_parameter('temperature_charge_max').value
        self.temperature_decharge_min = self.get_parameter('temperature_decharge_min').value
        self.temperature_decharge_max = self.get_parameter('temperature_decharge_max').value

        # Marge d'alerte préventive : alerte N°C avant la limite absolue
        self.declare_parameter('marge_alerte_temperature', 5.0)
        self.marge_alerte_temperature = self.get_parameter('marge_alerte_temperature').value

        # -------------------------------------------------------------------
        # Variables d'état interne (mises à jour à chaque cycle CAN)
        # -------------------------------------------------------------------
        self.instant_debut_pic_courant = None  # None = pas de pic en cours
        self.flags_protections_actives = 0     # bits de protection matérielle
        self.etat_mosfets              = 0     # bit0=MOS charge, bit1=MOS décharge
        self.temperatures_sondes       = []    # toutes les températures NTC (°C)
        self.nombre_cellules_en_serie  = 0     # lu depuis le registre 0x104
        self.nombre_sondes_ntc         = 0     # lu depuis le registre 0x104

        # -------------------------------------------------------------------
        # Message BatteryState ROS2 (réutilisé et mis à jour à chaque cycle)
        # -------------------------------------------------------------------
        self.etat_batterie = BatteryState()
        self.etat_batterie.header.frame_id = 'battery_link'
        self.etat_batterie.power_supply_technology = (
            BatteryState.POWER_SUPPLY_TECHNOLOGY_LIFEPO4
        )

        # -------------------------------------------------------------------
        # Table de correspondance : identifiant CAN → fonction de décodage
        # -------------------------------------------------------------------
        self.decodeurs_par_id_can = {
            0x100: self._decoder_tension_courant_capacite,
            0x101: self._decoder_soc_et_cycles,
            0x102: self._decoder_protections_et_balance,
            0x103: self._decoder_mosfets_et_version,
            0x104: self._decoder_configuration_batterie,
            0x105: self._decoder_temperatures_ntc_1_a_3,
            0x106: self._decoder_temperatures_ntc_4_a_6,
        }

        # -------------------------------------------------------------------
        # Connexion au bus CAN
        # -------------------------------------------------------------------
        try:
            self.bus_can = can.interface.Bus(
                channel='can0',
                bustype='socketcan',
                bitrate=500_000  # 500 kHz — imposé par la datasheet
            )
            self.get_logger().info("Bus CAN connecté. Nœud BMS démarré.")
        except Exception as erreur:
            self.get_logger().error(f"Impossible d'ouvrir le bus CAN : {erreur}")
            raise

        # -------------------------------------------------------------------
        # Publishers ROS2
        # -------------------------------------------------------------------
        self.pub_etat_batterie    = self.create_publisher(BatteryState, 'battery_state', 10)
        self.pub_commande_energie = self.create_publisher(String, 'robot_core/power_management', 10)
        self.pub_arret_urgence    = self.create_publisher(Bool,   'hardware/emergency_stop', 10)

        # Boucle principale à 2 Hz (toutes les 500 ms)
        self.create_timer(0.5, self._boucle_principale)

    # =======================================================================
    # VÉRIFICATION CRC-16
    # Permet de détecter les corruptions de données sur le bus CAN.
    # Algorithme : CRC-16/ARC (polynôme 0xA001, init 0xFFFF)
    # Source : datasheet §4 — big-endian uniquement
    # =======================================================================

    def _verifier_integrite_trame(self, trame_8_octets: bytes) -> bool:
        """
        Recalcule le CRC-16 sur les 6 premiers octets et le compare
        aux 2 derniers octets (signature envoyée par le BMS).
        Retourne True si les données sont intactes, False si corrompues.
        """
        if len(trame_8_octets) < 8:
            return False

        crc_calcule = 0xFFFF
        for octet in trame_8_octets[0:6]:
            crc_calcule ^= octet
            for _ in range(8):
                if crc_calcule & 0x0001:
                    crc_calcule = (crc_calcule >> 1) ^ 0xA001
                else:
                    crc_calcule >>= 1

        # Datasheet §2 : "high byte first" → big-endian
        crc_recu = struct.unpack('>H', trame_8_octets[6:8])[0]
        return crc_calcule == crc_recu

    # =======================================================================
    # DÉCODEURS CAN
    # =======================================================================

    def _decoder_tension_courant_capacite(self, octets: bytes):
        """
        Registre 0x100
          BYTE0~1 : tension totale du pack    (unité : 10 mV, non signé)
          BYTE2~3 : courant                   (unité : 10 mA, signé — + charge, − décharge)
          BYTE4~5 : capacité restante en mAh  (ignorée ici)
        """
        tension_brute, courant_brut, _ = struct.unpack('>HhH', octets[0:6])
        self.etat_batterie.voltage = tension_brute * 10.0 / 1000.0  # → Volts
        self.etat_batterie.current = courant_brut  * 10.0 / 1000.0  # → Ampères

    def _decoder_soc_et_cycles(self, octets: bytes):
        """
        Registre 0x101
          BYTE0~1 : capacité à pleine charge  (unité : 10 mAh)
          BYTE2~3 : nombre de cycles de décharge
          BYTE4~5 : RSOC — pourcentage de charge restant (0–100 %)
        """
        capacite_pleine_brute, _nombre_cycles, pourcentage_brut = (
            struct.unpack('>HHH', octets[0:6])
        )
        self.etat_batterie.capacity   = capacite_pleine_brute * 10.0 / 1000.0  # → Ah
        self.etat_batterie.percentage = pourcentage_brut / 100.0                # → 0.0–1.0

    def _decoder_protections_et_balance(self, octets: bytes):
        """
        Registre 0x102
          BYTE0~1 : flags d'équilibrage cellules 1 à 16
          BYTE2~3 : flags d'équilibrage cellules 17 à 33
          BYTE4~5 : flags de protection (16 bits, voir DESCRIPTIONS_PROTECTIONS)
        """
        _balance_basses, _balance_hautes, flags_protection = (
            struct.unpack('>HHH', octets[0:6])
        )

        anciens_flags = self.flags_protections_actives
        self.flags_protections_actives = flags_protection

        # On logue uniquement si l'état a changé depuis le cycle précédent
        if flags_protection != anciens_flags:
            protections_actives = [
                description
                for bit, description in DESCRIPTIONS_PROTECTIONS.items()
                if flags_protection & (1 << bit)
            ]
            if protections_actives:
                self.get_logger().error(
                    "PROTECTION HW active : " + ", ".join(protections_actives)
                )
            else:
                self.get_logger().info("Toutes les protections HW sont désactivées.")

    def _decoder_mosfets_et_version(self, octets: bytes):
        """
        Registre 0x103
          BYTE0~1 : état des MOSFETs (bit0 = MOS charge, bit1 = MOS décharge)
          BYTE2~3 : date de production (format compressé, datasheet §3 Note 3)
          BYTE4~5 : version du firmware
        """
        etat_mosfets, date_production_brute, version_firmware = (
            struct.unpack('>HHH', octets[0:6])
        )
        self.etat_mosfets = etat_mosfets

        if not (etat_mosfets & 0x02):
            self.get_logger().warn(
                "MOSFET de décharge OUVERT — le BMS bloque lui-même la décharge."
            )
        if not (etat_mosfets & 0x01):
            self.get_logger().warn(
                "MOSFET de charge OUVERT — le BMS bloque lui-même la charge."
            )

        # Décodage de la date de production (datasheet §3 Note 3)
        jour  =  date_production_brute & 0x1F
        mois  = (date_production_brute >> 5) & 0x0F
        annee =  2000 + (date_production_brute >> 9)
        self.get_logger().debug(
            f"BMS firmware v{version_firmware}, fabriqué le {jour:02d}/{mois:02d}/{annee}"
        )

    def _decoder_configuration_batterie(self, octets: bytes):
        """
        Registre 0x104 — lu une seule fois au premier cycle.
          BYTE0 : nombre de cellules en série
          BYTE1 : nombre de sondes NTC installées
        Note : trame 2+2 octets (2 données + 2 CRC), pas 6+2.
        """
        self.nombre_cellules_en_serie = octets[0]
        self.nombre_sondes_ntc        = octets[1]
        self.get_logger().info(
            f"Config BMS : {self.nombre_cellules_en_serie} cellules en série, "
            f"{self.nombre_sondes_ntc} sondes NTC"
        )

    def _decoder_temperatures_ntc_1_a_3(self, octets: bytes):
        """
        Registre 0x105 — Sondes NTC 1, 2, 3.
        Encodage datasheet §3 Note 4 :
          valeur_envoyée = 2731 + (température_celsius × 10)
          → température_celsius = (valeur − 2731) / 10
        Exemples :
          2981 → 25.0 °C  |  2631 → −10.0 °C
        """
        valeurs_brutes = struct.unpack('>HHH', octets[0:6])
        temperatures_ntc_1_a_3 = [(v - 2731) / 10.0 for v in valeurs_brutes]

        # Remplace les 3 premières entrées, conserve NTC4~6 si déjà lus
        self.temperatures_sondes = temperatures_ntc_1_a_3 + self.temperatures_sondes[3:]
        self._mettre_a_jour_temperature_batterystate()

    def _decoder_temperatures_ntc_4_a_6(self, octets: bytes):
        """
        Registre 0x106 — Sondes NTC 4, 5, 6 (absentes sur certains BMS).
        Même encodage que 0x105. Les valeurs nulles indiquent une sonde absente.
        """
        valeurs_brutes = struct.unpack('>HHH', octets[0:6])
        temperatures_ntc_4_a_6 = [
            (v - 2731) / 10.0 for v in valeurs_brutes if v != 0
        ]

        # S'assure qu'on a bien 3 entrées pour NTC1~3 avant d'ajouter NTC4~6
        while len(self.temperatures_sondes) < 3:
            self.temperatures_sondes.append(25.0)

        self.temperatures_sondes = self.temperatures_sondes[:3] + temperatures_ntc_4_a_6
        self._mettre_a_jour_temperature_batterystate()

    def _mettre_a_jour_temperature_batterystate(self):
        """
        BatteryState.temperature est un scalaire.
        On y met la température la plus élevée (la plus critique).
        """
        if self.temperatures_sondes:
            self.etat_batterie.temperature = max(self.temperatures_sondes)

    # =======================================================================
    # BOUCLE PRINCIPALE — appelée toutes les 500 ms par ROS2
    # =======================================================================

    def _boucle_principale(self):
        self.etat_batterie.header.stamp = self.get_clock().now().to_msg()

        self._lire_toutes_les_donnees_can()

        # On attend d'avoir au moins une mesure de tension valide
        if self.etat_batterie.voltage > 0:
            temperature_maximale = (
                max(self.temperatures_sondes) if self.temperatures_sondes else 25.0
            )
            temperature_minimale = (
                min(self.temperatures_sondes) if self.temperatures_sondes else 25.0
            )
            en_charge = self.etat_batterie.current > 0

            self._surveiller_et_alerter(en_charge, temperature_maximale, temperature_minimale)
            self._verifier_limites_et_couper(en_charge, temperature_maximale, temperature_minimale)

    # =======================================================================
    # LECTURE CAN
    # =======================================================================

    def _lire_toutes_les_donnees_can(self):
        """
        Pour chaque identifiant CAN :
          1. Envoie une remote frame (demande)
          2. Attend la réponse (max 50 ms)
          3. Vérifie l'intégrité CRC
          4. Décode avec la fonction correspondante
        Le registre 0x104 (config statique) n'est lu qu'au premier cycle.
        """
        identifiants_a_interroger = list(self.decodeurs_par_id_can.keys())

        if self.nombre_cellules_en_serie > 0:
            identifiants_a_interroger.remove(0x104)

        for id_registre in identifiants_a_interroger:
            demande = can.Message(
                arbitration_id=id_registre,
                is_remote_frame=True,
                is_extended_id=False
            )
            try:
                self.bus_can.send(demande)
                reponse = self.bus_can.recv(timeout=0.05)

                reponse_valide = (
                    reponse is not None
                    and not reponse.is_remote_frame
                    and reponse.arbitration_id == id_registre
                )

                if reponse_valide:
                    octets_bruts = bytes(reponse.data)
                    if self._verifier_integrite_trame(octets_bruts):
                        self.decodeurs_par_id_can[id_registre](octets_bruts)
                    else:
                        self.get_logger().warn(
                            f"Trame corrompue rejetée (registre 0x{id_registre:03X})"
                        )

            except can.CanError as erreur_can:
                self.get_logger().debug(
                    f"Erreur bus CAN sur registre 0x{id_registre:03X} : {erreur_can}"
                )

        self.pub_etat_batterie.publish(self.etat_batterie)

    # =======================================================================
    # SUPERVISION — alertes préventives (logs uniquement, aucune coupure)
    # =======================================================================

    def _surveiller_et_alerter(
        self,
        en_charge: bool,
        temperature_maximale: float,
        temperature_minimale: float
    ):
        tension = self.etat_batterie.voltage

        if en_charge:
            if tension > self.tension_charge_maximale:
                self.get_logger().warn(
                    f"SUPERVISION : Tension de charge élevée ({tension:.1f} V)"
                )
            if temperature_maximale >= self.temperature_charge_max - self.marge_alerte_temperature:
                self.get_logger().warn(
                    f"SUPERVISION : Surchauffe imminente en charge ({temperature_maximale:.1f}°C)"
                )
            if temperature_minimale <= self.temperature_charge_min + self.marge_alerte_temperature:
                self.get_logger().warn(
                    f"SUPERVISION : Température trop basse pour charger ({temperature_minimale:.1f}°C)"
                )
        else:
            if temperature_maximale >= self.temperature_decharge_max - self.marge_alerte_temperature:
                self.get_logger().warn(
                    f"SUPERVISION : Surchauffe imminente en décharge ({temperature_maximale:.1f}°C)"
                )

        if self.etat_mosfets and not (self.etat_mosfets & 0x02):
            self.get_logger().error(
                "SUPERVISION : MOSFET de décharge OUVERT — décharge bloquée par le BMS"
            )

    # =======================================================================
    # CONTRÔLE — limites absolues et déclenchement de l'E-STOP
    # =======================================================================

    def _verifier_limites_et_couper(
        self,
        en_charge: bool,
        temperature_maximale: float,
        temperature_minimale: float
    ):
        """
        Vérifie les 4 règles dans l'ordre de priorité.
        Dès qu'une règle déclenche un arrêt, on publie l'E-STOP et on sort (return).
        Si aucune règle n'est violée, on publie l'état nominal.
        """
        commande      = String()
        arret_urgence = Bool()
        arret_urgence.data = False

        # -------------------------------------------------------------------
        # Règle 1 : Température hors plage
        # -------------------------------------------------------------------
        if en_charge:
            hors_plage = (
                temperature_minimale < self.temperature_charge_min
                or temperature_maximale > self.temperature_charge_max
            )
        else:
            hors_plage = (
                temperature_minimale < self.temperature_decharge_min
                or temperature_maximale > self.temperature_decharge_max
            )

        if hors_plage:
            mode = "charge" if en_charge else "décharge"
            self.get_logger().fatal(
                f"CONTRÔLE : Température hors plage en {mode} "
                f"({temperature_minimale:.1f}°C ~ {temperature_maximale:.1f}°C). COUPURE."
            )
            self._publier_arret_urgence(commande, arret_urgence, "SHUTDOWN_TEMPERATURE")
            return

        # -------------------------------------------------------------------
        # Règle 2 : Tension trop basse (décharge uniquement)
        # -------------------------------------------------------------------
        if not en_charge and self.etat_batterie.voltage <= self.tension_coupure_basse:
            self.get_logger().fatal(
                f"CONTRÔLE : Tension critique ({self.etat_batterie.voltage:.2f} V). "
                f"COUPURE MOTEURS."
            )
            self._publier_arret_urgence(commande, arret_urgence, "SHUTDOWN_TENSION_BASSE")
            return

        # -------------------------------------------------------------------
        # Règle 3 : Pic de courant tenu trop longtemps
        # -------------------------------------------------------------------
        courant_decharge_absolu = abs(self.etat_batterie.current) if not en_charge else 0.0

        if courant_decharge_absolu > self.courant_pic_maximal:
            if self.instant_debut_pic_courant is None:
                self.instant_debut_pic_courant = self.get_clock().now()
                self.get_logger().warn(
                    f"CONTRÔLE : Pic de courant détecté ({courant_decharge_absolu:.1f} A). "
                    f"Toléré {self.duree_pic_maximale:.0f} s."
                )
            else:
                duree_pic_secondes = (
                    self.get_clock().now() - self.instant_debut_pic_courant
                ).nanoseconds / 1e9

                if duree_pic_secondes > self.duree_pic_maximale:
                    self.get_logger().fatal(
                        f"CONTRÔLE : Pic maintenu depuis {duree_pic_secondes:.1f} s. COUPURE."
                    )
                    self._publier_arret_urgence(commande, arret_urgence, "SHUTDOWN_PIC_COURANT")
                    return
        else:
            if self.instant_debut_pic_courant is not None:
                self.get_logger().info("CONTRÔLE : Pic de courant terminé. Retour normal.")
            self.instant_debut_pic_courant = None

        # -------------------------------------------------------------------
        # Règle 4 : Aucune limite franchie — état nominal
        # -------------------------------------------------------------------
        if en_charge:
            commande.data = "EN_CHARGE"
        elif self.etat_batterie.voltage <= self.tension_alerte_basse:
            commande.data = "BATTERIE_FAIBLE"
        else:
            commande.data = "FONCTIONNEMENT_NORMAL"

        self.pub_commande_energie.publish(commande)
        self.pub_arret_urgence.publish(arret_urgence)

    # =======================================================================
    # UTILITAIRE
    # =======================================================================

    def _publier_arret_urgence(
        self,
        commande: String,
        arret_urgence: Bool,
        code_arret: str
    ):
        """
        Publie simultanément la commande d'arrêt ET le signal E-STOP.
        Les deux publications sont groupées pour éviter tout état incohérent.
        """
        commande.data      = code_arret
        arret_urgence.data = True
        self.pub_commande_energie.publish(commande)
        self.pub_arret_urgence.publish(arret_urgence)


# ===========================================================================
# POINT D'ENTRÉE
# ===========================================================================

def main(args=None):
    rclpy.init(args=args)
    noeud = NoeudGestionBatterie()
    try:
        rclpy.spin(noeud)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()