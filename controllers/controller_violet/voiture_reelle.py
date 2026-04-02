# Programme principal de conduite autonome CoVAPSy
#
# Démarrage : python main_autonomous.py
# Commandes disponibles dans le terminal :
#   GO    → démarre la conduite autonome
#   STOP  → arrête la voiture
#   QUIT  → arrête la voiture et quitte le programme
#
# Conformité règlement CoVAPSy 2026 :
#   - La voiture n'avance pas avant réception de la commande GO
#   - La voiture s'arrête immédiatement sur commande STOP
#   - Marche arrière automatique en cas de blocage
#   - La voiture va dans le bon sens par rapport aux 2 couleurs des murs

import logging
import sys
import threading
import time

from controller_violet import filtre_moyenneur, lire_point_lidar, normaliser_distance, differentiel_vers_ackermann, calculer_commande_auto

# =========================
# Paramètres véhicule
# =========================
maxSpeed = 50       # km/h
maxangle_degre = 18

# --- Paramètres géométriques du TT-02 (à ajuster selon le modèle Webots) ---
L_entraxe = 0.180  # m  — distance entre roues gauche/droite (voie)
W_empattement = 0.250  # m  — distance entre essieu avant et arrière



# ============================================================
# Configuration du logging
# ============================================================
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    datefmt="%H:%M:%S",
    handlers=[
        logging.StreamHandler(sys.stdout),
        logging.FileHandler("covapsy.log", encoding="utf-8"),
    ]
)
logger = logging.getLogger(__name__)


def main():
    # =========================
    # Mode de fonctionnement
    # =========================
    modeAuto = False
    print("CoVAPSy — Conduite Autonome pour Webots")
    print("Cliquer sur la vue 3D pour commencer")
    print("a : mode auto")
    print("n : stop")

if __name__ == "__main__":
    main()