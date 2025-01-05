from dynamixel_sdk import *  # Importer la bibliothèque Dynamixel SDK
import msvcrt
from numpy import abs as absolue
from numpy import pi as pi
import time

def getch():
    return msvcrt.getch().decode()

def convert_to_signed_32bit(value):
    if value > 0x7FFFFFFF:  # Si la valeur dépasse le maximum d'un entier signé 32 bits
        value -= 0x100000000  
    return value

def maxVit(v):
    if v>445 :
        return 445
    elif v<-445 :
        return -445
    else :
        return v
    
    
def encode_velocity(value):
    if value < 0:
        value = (1 << 32) + value  
    return value

def convert_to_signed(value, bit_width):
    """Convertit une valeur brute non signée en une valeur signée."""
    max_value = 1 << (bit_width - 1)
    if value >= max_value:
        value -= (1 << bit_width)
    return value

def rota_vitesse(v):
    return v * 0.229 * 2 * pi * 4 / 60

def vitesse_rota(v):
    return v / (0.229 * 2 * pi * 4 / 60)


# Paramètres du port
PORT_NAME = "COM5"  # Remplacez par le port correct sur votre machine
BAUDRATE = 57600
PROTOCOL_VERSION = 2.0

# Adresses des registres de contrôle
ADDR_TORQUE_ENABLE = 64 # Adresses du XL330
ADDR_OPERATING_MODE = 11
ADDR_GOAL_VELOCITY = 104
ADDR_PRESENT_VELOCITY = 128
ADDR_PRESENT_CURRENT = 126
ADDR_PRESENT_VOLTAGE = 144

# Paramètres Dynamixel
DXL_ID = 1
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
MODE_VELOCITY = 1  # Mode de fonctionnement : vitesse
GOAL_VELOCITY = 0  # Vitesse cible (valeur Dynamixel, essayez une valeur plus petite si nécessaire)

# Initialisation du port et du gestionnaire de paquets
port_handler = PortHandler(PORT_NAME)
packet_handler = PacketHandler(PROTOCOL_VERSION)    

# Ouvrir le port
if not port_handler.openPort():
    print("Erreur : Impossible d'ouvrir le port")
    quit()

# Configurer le baudrate
if not port_handler.setBaudRate(BAUDRATE):
    print("Erreur : Impossible de configurer le baudrate")
    port_handler.closePort()
    quit()

# Désactiver le couple avant la configuration
dxl_comm_result, dxl_error = packet_handler.write1ByteTxRx(port_handler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print(f"Erreur désactivation couple : {packet_handler.getTxRxResult(dxl_comm_result)}")
    port_handler.closePort()
    quit()

# Configurer en mode vitesse
dxl_comm_result, dxl_error = packet_handler.write1ByteTxRx(port_handler, DXL_ID, ADDR_OPERATING_MODE, MODE_VELOCITY)
if dxl_comm_result != COMM_SUCCESS:
    print(f"Erreur configuration mode : {packet_handler.getTxRxResult(dxl_comm_result)}")
    port_handler.closePort()
    quit()

# Activer le couple
dxl_comm_result, dxl_error = packet_handler.write1ByteTxRx(port_handler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print(f"Erreur activation couple : {packet_handler.getTxRxResult(dxl_comm_result)}")
    port_handler.closePort()
    quit()

# Initialisation des variables pour la puissance
P_moyenne = 0
moyenne_status = 0

try:
    while True:

        if msvcrt.kbhit():  # Vérifie si une touche est en attente
            key = getch()
            if key == 'q':  # Quitter la boucle si 'q' est pressé
                break
            elif key == '+':  # Augmenter la vitesse
                GOAL_VELOCITY = maxVit(GOAL_VELOCITY + 10)
            elif key == '-':  # Diminuer la vitesse
                GOAL_VELOCITY  = maxVit(GOAL_VELOCITY - 10)
            elif key == '0':  # Stopper le moteur
                GOAL_VELOCITY = 0
            elif key == '/': # Inverser la direction
                GOAL_VELOCITY = -GOAL_VELOCITY
            elif key == '9': # Vitesse max
                GOAL_VELOCITY = 445
            elif key == '1': # Vitesse max sens inverse
                GOAL_VELOCITY = -445
            elif key == 'a' : # 5 mm/s
                GOAL_VELOCITY = int(vitesse_rota(5))
            elif key == 'z' : # 10 mm/s
                GOAL_VELOCITY = int(vitesse_rota(10))
            elif key == 'e' : # 15 mm/s
                GOAL_VELOCITY = int(vitesse_rota(15))
            elif key == 'r' : # 20 mm/s
                GOAL_VELOCITY = int(vitesse_rota(20))
            elif key == 't' : # 25 mm/s
                GOAL_VELOCITY = int(vitesse_rota(25))
            elif key == 'y' : # 30 mm/s
                GOAL_VELOCITY = int(vitesse_rota(30))
            elif key == 'u' : # 35 mm/s
                GOAL_VELOCITY = int(vitesse_rota(35))
            elif key == 'i' : # 40 mm/s
                GOAL_VELOCITY = int(vitesse_rota(40))
            elif key == 'o' : # 45 mm/s
                GOAL_VELOCITY = int(vitesse_rota(45))
            elif key == 'm' : # Moyennage de la puissance
                moyenne_status = 1
                P_moyenne = 0
                P_moyenne = P_moyenne + P_instant
            elif key == 'n' : # Arrêt du moyennage
                moyenne_status = 0


            
        
        # Fixer la vitesse cible
        dxl_comm_result, dxl_error = packet_handler.write4ByteTxRx(port_handler, DXL_ID, ADDR_GOAL_VELOCITY, encode_velocity(GOAL_VELOCITY))
        if dxl_comm_result != COMM_SUCCESS:
            print(f"Erreur fixation vitesse : {packet_handler.getTxRxResult(dxl_comm_result)}")
            break

        #Lire l'intensité
        dxl_present_current, dxl_comm_result, dxl_error = packet_handler.read2ByteTxRx(port_handler, DXL_ID, ADDR_PRESENT_CURRENT) # en mA
        if dxl_comm_result != COMM_SUCCESS:
            print(f"Erreur lecture intensité : {packet_handler.getTxRxResult(dxl_comm_result)}")
        else :
            dxl_present_current = convert_to_signed(dxl_present_current, 16)
        
        #Lire la tension
        dxl_present_voltage, dxl_comm_result, dxl_error = packet_handler.read2ByteTxRx(port_handler, DXL_ID, ADDR_PRESENT_VOLTAGE)
        if dxl_comm_result != COMM_SUCCESS:
            print(f"Erreur lecture tension : {packet_handler.getTxRxResult(dxl_comm_result)}")
        else :
            dxl_present_voltage = dxl_present_voltage * 0.1 # Conversion en Volt

        #Calcul de la puissance
        # Calcul de la puissance instantanée
        P_instant = dxl_present_current * dxl_present_voltage  # en mW
        P_instant = absolue(P_instant)  # Valeur absolue de la puissance

        # Moyennage de la puissance
        if moyenne_status == 1:
            P_moyenne = (P_moyenne + P_instant) / 2
        
        P = P_moyenne
        

        # Lire la vitesse actuelle
        dxl_present_velocity, dxl_comm_result, dxl_error = packet_handler.read4ByteTxRx(port_handler, DXL_ID, ADDR_PRESENT_VELOCITY)
        if dxl_comm_result != COMM_SUCCESS:
            print(f"Erreur lecture vitesse : {packet_handler.getTxRxResult(dxl_comm_result)}")
        else:
            dxl_present_velocity = convert_to_signed_32bit(dxl_present_velocity)
            dxl_present_velocity = rota_vitesse(dxl_present_velocity) # Conversion en mm/s
            print(f"\rV = {dxl_present_velocity:.3f} | Vc = {rota_vitesse(GOAL_VELOCITY):.3f} | P = {P:.3f}, U = {dxl_present_voltage:.3f}, I = {dxl_present_current:.3f}        ", end="")
        
        
           
        
finally:
    # Arrêter le moteur
    dxl_comm_result, dxl_error = packet_handler.write2ByteTxRx(port_handler, DXL_ID, ADDR_GOAL_VELOCITY, 0)
    if dxl_comm_result != COMM_SUCCESS:
        print(f"Erreur arrêt moteur : {packet_handler.getTxRxResult(dxl_comm_result)}")

    # Désactiver le couple
    dxl_comm_result, dxl_error = packet_handler.write1ByteTxRx(port_handler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print(f"Erreur désactivation couple : {packet_handler.getTxRxResult(dxl_comm_result)}")

    # Fermer le port
    port_handler.closePort()
    print("\nPort fermé.")