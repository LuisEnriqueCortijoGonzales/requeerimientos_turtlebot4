import socket
import time
from typing import Tuple

# =================== CONFIGURACIÓN DE RED ===================
TURTLEBOT_IP   = "10.153.100.177"   # IP del TurtleBot4
TURTLEBOT_PORT = 5007              # Mismo puerto que el receptor

SEND_HZ = 20   # Frecuencia de envío de comandos (20 Hz)

# =================== IDENTIDAD / SEGURIDAD ===================
DESIRED_DOMAIN_ID = 21           # ROS_DOMAIN_ID que esperas que tenga ese robot
PAIRING_CODE      = "ROBOT_A_42" # Código único compartido con ese robot

# =================== LIMITES DE SEGURIDAD ===================
MAX_LIN_VEL = 0.26   # m/s
MAX_ANG_VEL = 1.0    # rad/s


# =================== FUNCIÓN QUE TÚ DEBES RELLENAR ===================
def get_control_command() -> Tuple[float, float]:
    """
    Aquí conectas lo que quieras:
      - modelo de ML
      - mando
      - teclas
      - etc.

    Debe devolver (vx, wz) en unidades SI.
    """
    # EJEMPLO DEMO: círculo constante
    vx = 0.1   # m/s
    wz = 0.3   # rad/s
    return vx, wz


# =================== UTILIDADES ===================
def clamp(value: float, min_val: float, max_val: float) -> float:
    return max(min(value, max_val), min_val)


def do_handshake(sock: socket.socket, robot_addr) -> bool:
    """
    Realiza el handshake con el robot:
      Envía:  HELLO <DESIRED_DOMAIN_ID> <PAIRING_CODE>
      Espera: ACK <domain_id> <robot_name>
    """
    sock.settimeout(1.0)  # 1 s de timeout para recepción

    print(f"[HANDSHAKE] Iniciando con {robot_addr}...")
    while True:
        # 1) Enviar HELLO
        msg = f"HELLO {DESIRED_DOMAIN_ID} {PAIRING_CODE}".encode("utf-8")
        sock.sendto(msg, robot_addr)

        try:
            # 2) Esperar ACK
            data, addr = sock.recvfrom(1024)
            text = data.decode("utf-8").strip()
            parts = text.split()

            if len(parts) >= 3 and parts[0] == "ACK":
                domain_str = parts[1]
                robot_name = " ".join(parts[2:])

                print(f"[HANDSHAKE] Respuesta desde {addr}: {text}")

                try:
                    domain_id = int(domain_str)
                except ValueError:
                    print("[HANDSHAKE] domain_id inválido en ACK, reintentando...")
                    continue

                if domain_id != DESIRED_DOMAIN_ID:
                    print(f"[HANDSHAKE] ROS_DOMAIN_ID no coincide "
                          f"(esperado={DESIRED_DOMAIN_ID}, recibido={domain_id}). Reintentando...")
                    continue

                print(f"[HANDSHAKE] Emparejado con robot '{robot_name}' (ROS_DOMAIN_ID={domain_id}).")
                return True

            else:
                print(f"[HANDSHAKE] Mensaje inesperado: '{text}', reintentando...")

        except socket.timeout:
            print("[HANDSHAKE] Timeout esperando ACK, reintentando...")

        except KeyboardInterrupt:
            print("\n[HANDSHAKE] Cancelado por el usuario.")
            return False


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    robot_addr = (TURTLEBOT_IP, TURTLEBOT_PORT)

    # 1) Handshake antes de mandar comandos
    ok = do_handshake(sock, robot_addr)
    if not ok:
        print("[MAIN] Handshake fallido. Saliendo.")
        sock.close()
        return

    dt = 1.0 / SEND_HZ
    print(f"[MAIN] Enviando comandos CMD vx wz a {robot_addr} a {SEND_HZ} Hz. Ctrl+C para detener.")

    try:
        while True:
            vx, wz = get_control_command()

            vx = clamp(vx, -MAX_LIN_VEL, MAX_LIN_VEL)
            wz = clamp(wz, -MAX_ANG_VEL, MAX_ANG_VEL)

            msg = f"CMD {vx:.4f} {wz:.4f}".encode("utf-8")
            sock.sendto(msg, robot_addr)

            time.sleep(dt)

    except KeyboardInterrupt:
        print("\n[MAIN] Parando robot...")
        stop_msg = "CMD 0.0 0.0".encode("utf-8")
        sock.sendto(stop_msg, robot_addr)
        sock.close()
        print("[MAIN] Socket cerrado.")


if __name__ == "__main__":
    main()
