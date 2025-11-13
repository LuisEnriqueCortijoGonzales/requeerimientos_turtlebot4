import socket
import time
from typing import Tuple

# =================== CONFIGURACIÓN DE RED ===================
TURTLEBOT_IP   = "10.153.100.177"  # <-- Cambiar por la IP del TurtleBot4
TURTLEBOT_PORT = 5007             # <-- Debe coincidir con el puerto del receptor en el robot

SEND_HZ = 20   # Frecuencia de envío de comandos (20 Hz ~ cada 50 ms)

# =================== LIMITES DE SEGURIDAD ===================
MAX_LIN_VEL = 0.26   # m/s (valor típico seguro para TurtleBot)
MAX_ANG_VEL = 1.0    # rad/s


# =================== FUNCIÓN QUE TÚ DEBES RELLENAR ===================
def get_control_command() -> Tuple[float, float]:
    """
    Esta función es el corazón de la plantilla.
    Aquí debes leer tu fuente de control:
      - Mando (joystick)
      - Modelo de ML / deep learning
      - Algoritmo de navegación
      - Teclado, etc.

    Debe devolver:
      vx: velocidad lineal (m/s)
      wz: velocidad angular (rad/s)
    """
    # ===== EJEMPLOS DE USO (descomenta uno y adapta) =====

    # 1) EJEMPLO: leer de un modelo ML
    # vx, wz = mi_modelo.predict(estado_actual)

    # 2) EJEMPLO: mando (si ya tienes funciones para leer joystick)
    # vx, wz = leer_mando()

    # 3) EJEMPLO: demo tonta, que hace girar en círculo
    vx = 0.1   # avanza lento
    wz = 0.3   # gira suave

    return vx, wz


# =================== UTILIDADES ===================
def clamp(value: float, min_val: float, max_val: float) -> float:
    return max(min(value, max_val), min_val)


def main():
    # Crear socket UDP
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    dt = 1.0 / SEND_HZ
    print(f"Enviando comandos a {TURTLEBOT_IP}:{TURTLEBOT_PORT} a {SEND_HZ} Hz")
    print("Ctrl+C para detener.")

    try:
        while True:
            # 1) Obtener comando desde tu fuente (ML, mando, etc.)
            vx, wz = get_control_command()

            # 2) Aplicar límites de seguridad
            vx = clamp(vx, -MAX_LIN_VEL, MAX_LIN_VEL)
            wz = clamp(wz, -MAX_ANG_VEL, MAX_ANG_VEL)

            # 3) Serializar al protocolo "vx,wz"
            msg = f"{vx:.4f},{wz:.4f}".encode("utf-8")

            # 4) Enviar por UDP al TurtleBot
            sock.sendto(msg, (TURTLEBOT_IP, TURTLEBOT_PORT))

            # 5) Esperar hasta el siguiente ciclo
            time.sleep(dt)

    except KeyboardInterrupt:
        print("\n[INFO] Controlador detenido. Enviando parada final...")
        # Enviar un último comando de parada
        stop_msg = "0.0,0.0".encode("utf-8")
        sock.sendto(stop_msg, (TURTLEBOT_IP, TURTLEBOT_PORT))
        sock.close()
        print("[INFO] Socket cerrado. Robot detenido (si el receptor está activo).")


if __name__ == "__main__":
    main()
