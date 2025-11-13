#!/usr/bin/env python3
import os
import socket
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class UdpToCmdVel(Node):
    def __init__(self):
        super().__init__("udp_to_cmdvel")

        # ---------- Parámetros ----------
        self.declare_parameter("port", 5007)
        self.declare_parameter("robot_name", "turtlebot4_lite_1")
        self.declare_parameter("pairing_code", "ROBOT_A_42")  # debe coincidir con el cliente
        self.declare_parameter("topic_name", "/cmd_vel")

        port         = self.get_parameter("port").get_parameter_value().integer_value
        self.robot_name  = self.get_parameter("robot_name").get_parameter_value().string_value
        self.pairing_code = self.get_parameter("pairing_code").get_parameter_value().string_value
        topic_name   = self.get_parameter("topic_name").get_parameter_value().string_value

        # ---------- Leer ROS_DOMAIN_ID ----------
        self.ros_domain_id = int(os.environ.get("ROS_DOMAIN_ID", "0"))
        self.get_logger().info(f"ROS_DOMAIN_ID detectado: {self.ros_domain_id}")

        # ---------- Publisher ----------
        self.pub_cmd_vel = self.create_publisher(Twist, topic_name, 10)

        # ---------- Socket UDP ----------
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", port))
        self.get_logger().info(f"Escuchando UDP en 0.0.0.0:{port}")

        # ---------- Estado de emparejamiento ----------
        self.authorized_addr = None  # (ip, puerto) del controlador emparejado
        self.get_logger().info("Esperando handshake HELLO ...")

        # ---------- Watchdog ----------
        self.timeout_seconds = 0.5
        self.last_msg_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.watchdog_callback)

        # ---------- Hilo UDP ----------
        self.running = True
        self.thread = threading.Thread(target=self.udp_loop, daemon=True)
        self.thread.start()

    # ==========================================================
    #                      LÓGICA UDP
    # ==========================================================
    def udp_loop(self):
        self.get_logger().info("Hilo UDP iniciado.")
        while self.running:
            try:
                data, addr = self.sock.recvfrom(1024)
                text = data.decode("utf-8").strip()
                parts = text.split()

                if not parts:
                    continue

                cmd_type = parts[0]

                # ---------- Handshake ----------
                if cmd_type == "HELLO":
                    self.handle_hello(parts, addr)

                # ---------- Comando ----------
                elif cmd_type == "CMD":
                    self.handle_cmd(parts, addr)

                else:
                    self.get_logger().warn(f"Mensaje desconocido desde {addr}: '{text}'")

            except Exception as e:
                self.get_logger().error(f"Error en udp_loop: {e}")
                break

        self.get_logger().info("Hilo UDP finalizado.")

    # ---------- Manejo de HELLO ----------
    def handle_hello(self, parts, addr):
        # Formato: HELLO <desired_domain_id> <pairing_code>
        if len(parts) < 3:
            self.get_logger().warn(f"HELLO inválido desde {addr}: {parts}")
            return

        desired_domain_str = parts[1]
        pairing_code = parts[2]

        try:
            desired_domain = int(desired_domain_str)
        except ValueError:
            self.get_logger().warn(f"HELLO con domain_id inválido desde {addr}: '{desired_domain_str}'")
            return

        # Verificar pairing_code
        if pairing_code != self.pairing_code:
            self.get_logger().warn(f"HELLO con pairing_code incorrecto desde {addr}")
            return

        # Verificar domain_id
        if desired_domain != self.ros_domain_id:
            self.get_logger().warn(
                f"HELLO con domain_id {desired_domain} pero este robot tiene {self.ros_domain_id}"
            )
            return

        # Si pasa todo, aceptar emparejamiento (solo 1 controlador)
        if self.authorized_addr is None:
            self.authorized_addr = addr
            self.get_logger().info(f"Controlador emparejado: {addr}")
        else:
            # Si hay ya un emparejado y viene de otra IP, ignorar
            if addr != self.authorized_addr:
                self.get_logger().warn(
                    f"HELLO desde {addr} pero ya hay controlador emparejado: {self.authorized_addr}"
                )
                return

        # Responder ACK <domain_id> <robot_name>
        ack_msg = f"ACK {self.ros_domain_id} {self.robot_name}".encode("utf-8")
        self.sock.sendto(ack_msg, addr)

    # ---------- Manejo de CMD ----------
    def handle_cmd(self, parts, addr):
        # Ignorar comandos de direcciones no emparejadas
        if self.authorized_addr is None:
            self.get_logger().warn(f"CMD recibido desde {addr} antes de handshake. Ignorando.")
            return

        if addr != self.authorized_addr:
            # Esto evita que otro PC en la red controle el robot accidentalmente
            self.get_logger().warn(f"CMD desde {addr} pero autorizado es {self.authorized_addr}. Ignorando.")
            return

        # Formato: CMD <vx> <wz>
        if len(parts) < 3:
            self.get_logger().warn(f"CMD inválido desde {addr}: {parts}")
            return

        vx_str, wz_str = parts[1], parts[2]

        try:
            vx = float(vx_str)
            wz = float(wz_str)
        except ValueError:
            self.get_logger().warn(f"CMD con valores no numéricos desde {addr}: {parts}")
            return

        twist = Twist()
        twist.linear.x = vx
        twist.angular.z = wz
        self.pub_cmd_vel.publish(twist)
        self.last_msg_time = self.get_clock().now()

    # ==========================================================
    #                    WATCHDOG DE PARADA
    # ==========================================================
    def watchdog_callback(self):
        now = self.get_clock().now()
        elapsed = (now - self.last_msg_time).nanoseconds / 1e9
        if elapsed > self.timeout_seconds:
            twist = Twist()  # todo 0
            self.pub_cmd_vel.publish(twist)

    def destroy_node(self):
        self.running = False
        try:
            self.sock.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UdpToCmdVel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
