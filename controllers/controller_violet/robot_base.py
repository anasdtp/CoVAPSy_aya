# robot_base.py — Classes d'abstraction matérielle : actionneurs et lidar
import logging
import threading
import time

from rpi_hardware_pwm import HardwarePWM
from rplidar import RPLidar

import config

logger = logging.getLogger(__name__)


class Actionneurs:
    """Pilote la propulsion et la direction via HardwarePWM."""

    def __init__(self):
        self._pwm_prop = HardwarePWM(pwm_channel=0, hz=50)
        self._pwm_dir  = HardwarePWM(pwm_channel=1, hz=50)
        self._actif    = False

    def demarrer(self):
        """Active les deux PWM et place la voiture à l'arrêt, roues droites."""
        self._pwm_prop.start(config.PWM_STOP_PROP)
        self._pwm_dir.start(config.ANGLE_PWM_CENTRE)
        self._actif = True
        logger.info("Actionneurs démarrés")

    def arreter(self):
        """Remet à l'arrêt puis désactive les PWM."""
        if self._actif:
            try:
                self.set_vitesse_m_s(0)
                self.set_direction_degre(0)
            except Exception:
                pass
        self._pwm_prop.stop()
        self._pwm_dir.stop()
        self._actif = False
        logger.info("Actionneurs arrêtés")

    # ----------------------------------------------------------
    # Commandes de base
    # ----------------------------------------------------------

    def set_vitesse_m_s(self, vitesse_m_s: float):
        """Commande la vitesse en m/s. Positif = avant, négatif = arrière."""
        # Saturation logicielle
        vitesse_m_s = max(-config.VITESSE_MAX_M_S_HARD,
                          min(config.VITESSE_MAX_M_S_SOFT, vitesse_m_s))

        if vitesse_m_s == 0:
            self._pwm_prop.change_duty_cycle(config.PWM_STOP_PROP)
        elif vitesse_m_s > 0:
            # pwm_stop + direction * (point_mort + fraction_vitesse)
            v = vitesse_m_s * config.DELTA_PWM_MAX_PROP / config.VITESSE_MAX_M_S_HARD
            self._pwm_prop.change_duty_cycle(
                config.PWM_STOP_PROP + config.DIRECTION_PROP * (config.POINT_MORT_PROP + v)
            )
        else:
            # Marche arrière : soustraction symétrique
            v = vitesse_m_s * config.DELTA_PWM_MAX_PROP / config.VITESSE_MAX_M_S_HARD
            self._pwm_prop.change_duty_cycle(
                config.PWM_STOP_PROP - config.DIRECTION_PROP * (config.POINT_MORT_PROP - v)
            )

    def set_direction_degre(self, angle_degre: float):
        """Commande l'angle de braquage en degrés. 0 = tout droit.
        +angle_degre_max = gauche, -angle_degre_max = droite.
        """
        # Conversion degrés → duty cycle avec saturation sur les butées physiques
        angle_pwm = (
            config.ANGLE_PWM_CENTRE
            + config.DIRECTION_DIR
            * (config.ANGLE_PWM_MAX - config.ANGLE_PWM_MIN)
            * angle_degre / (2 * config.ANGLE_DEGRE_MAX)
        )
        angle_pwm = max(config.ANGLE_PWM_MIN, min(config.ANGLE_PWM_MAX, angle_pwm))
        self._pwm_dir.change_duty_cycle(angle_pwm)

    def recule(self):
        """Séquence de recul : impulsion courte arrière, pause, puis recul lent."""
        logger.info("Séquence de recul déclenchée")
        # Impulsion courte pour débloquer le variateur
        self.set_vitesse_m_s(-config.VITESSE_MAX_M_S_HARD)
        time.sleep(0.2)
        self.set_vitesse_m_s(0)
        time.sleep(0.2)
        # Recul lent
        self.set_vitesse_m_s(config.VITESSE_RECUL_M_S)
        time.sleep(config.DUREE_RECUL_S)
        self.set_vitesse_m_s(0)


class CapteurLidar:
    """Acquisition lidar dans un thread dédié.

    Usage :
        lidar = CapteurLidar()
        lidar.connecter()
        lidar.demarrer()
        ...
        if lidar.lire():          # retourne True si nouveau scan disponible
            tab = lidar.tableau_mm  # liste de 360 distances en mm
        ...
        lidar.arreter()
    """

    def __init__(self):
        self.tableau_mm: list = [0] * 360   # tableau public lu par la logique de conduite
        self._acqui_mm: list  = [0] * 360   # tampon d'acquisition (thread lidar)
        self._nouveau_scan    = False
        self._run             = False
        self._thread          = None
        self._lidar           = None
        self._lock            = threading.Lock()

    def connecter(self):
        """Ouvre la connexion série et démarre le moteur du lidar.

        Séquence identique à test_lidar.py qui fonctionne :
        connect() → get_info() → start_motor()
        """
        self._lidar = RPLidar(config.LIDAR_PORT, baudrate=config.LIDAR_BAUDRATE)
        self._lidar.connect()
        logger.info("Lidar connecté : %s", self._lidar.get_info())
        self._lidar.start_motor()
        time.sleep(1)

    def demarrer(self):
        """Lance le thread d'acquisition."""
        self._run    = True
        self._thread = threading.Thread(target=self._scan_loop, daemon=True, name="lidar_scan")
        self._thread.start()
        logger.info("Thread lidar démarré")

    def arreter(self):
        """Stoppe l'acquisition et déconnecte proprement le lidar."""
        self._run = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=3)
        if self._lidar:
            try:
                self._lidar.stop_motor()
                self._lidar.stop()
                time.sleep(1)
                self._lidar.disconnect()
            except Exception as e:
                logger.warning("Erreur à la déconnexion lidar : %s", e)
        logger.info("Lidar arrêté")

    def lire(self) -> bool:
        """Copie le tampon d'acquisition dans tableau_mm si un nouveau scan est prêt.

        Retourne True si un nouveau scan a été consommé, False sinon.
        Thread-safe.
        """
        with self._lock:
            if self._nouveau_scan:
                self.tableau_mm = self._acqui_mm.copy()
                self._nouveau_scan = False
                return True
        return False

    def _scan_loop(self):
        """Boucle interne du thread d'acquisition (exécutée dans le thread lidar)."""
        while self._run:
            try:
                for scan in self._lidar.iter_scans(scan_type='express'):
                    # Chaque 'scan' est une liste de tuples (quality, angle, distance_mm)
                    for _, angle, distance in scan:
                        # Le 0° physique du lidar pointe vers l'ARRIÈRE de la voiture.
                        # Les angles physiques entre -90° et +90° (soit 0-90° et 270-360°)
                        # visent l'intérieur/dessous de la voiture → on les ignore.
                        a = int(angle) % 360
                        if a <= 90 or a >= 270:
                            continue
                        # Correction : décalage de 180° pour que tableau_mm[0] = devant.
                        idx = (180 - a) % 360
                        self._acqui_mm[idx] = distance
                    with self._lock:
                        self._nouveau_scan = True
                    time.sleep(0.005)
                    if not self._run:
                        break
            except Exception as e:
                logger.warning("Erreur acquisition lidar : %s", e)
                time.sleep(0.1)


# ============================================================
# Test standalone
# ============================================================
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")

    lidar = CapteurLidar()
    lidar.connecter()
    lidar.demarrer()

    act = Actionneurs()
    act.demarrer()

    try:
        print("Test robot_base.py — Ctrl+C pour arrêter")
        print("Attente de 3 scans lidar...")
        nb = 0
        while nb < 3:
            if lidar.lire():
                print(f"Scan #{nb+1} — devant : {lidar.tableau_mm[0]:.0f} mm")
                nb += 1
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        act.arreter()
        lidar.arreter()
        print("Arrêt propre")
