import serial
import time

# Adapter le port si nécessaire
# Raspberry Pi : /dev/serial0 ou /dev/ttyAMA0
# USB-Serial : /dev/ttyUSB0
PORT = "/dev/tty.usbserial-655254C93F"
BAUD = 9600

print(f"Connexion au GPS sur {PORT} à {BAUD} bauds...")
print("En attente de données (Ctrl+C pour quitter)\n")

try:
    ser = serial.Serial(PORT, BAUD, timeout=2)
    time.sleep(1)

    trames = 0
    fix = False

    while True:
        line = ser.readline().decode('ascii', errors='replace').strip()

        if line:
            trames += 1
            print(line)

            # Vérifier si on a un fix GPS via la trame GPRMC
            if line.startswith('$GPRMC') or line.startswith('$GNRMC'):
                parts = line.split(',')
                if len(parts) > 2 and parts[2] == 'A':
                    fix = True
                    print(">>> FIX GPS OK ! <<<")

            # Résumé toutes les 20 trames
            if trames % 20 == 0:
                status = "FIX OK" if fix else "Pas de fix (normal en intérieur)"
                print(f"\n--- {trames} trames reçues | {status} ---\n")
        else:
            print("... aucune donnée reçue (vérifier câblage TX/RX)")

except serial.SerialException as e:
    print(f"Erreur port série : {e}")
    print("Vérifier : sudo raspi-config -> Interface Options -> Serial Port")
except KeyboardInterrupt:
    print(f"\nArrêt. Total : {trames} trames reçues.")
finally:
    if 'ser' in locals():
        ser.close()