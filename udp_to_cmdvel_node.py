#!/usr/bin/env python3
import socket
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class UdpToCmdVel(Node):
    def __init__(self):
        super().__init__("udp_to_cmdvel")

        # Parámetros
        self.declare_parameter("port", 5007)
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("topic_name", "/cmd_vel")

        port = self.get_parameter("port").get_parameter_value().integer_value
        topic_name = self.get_parameter("topic_name").get_parameter_value().string_value

        # Publisher a /cmd_vel
        self.pub_cmd_vel = self.create_publisher(Twist, topic_name, 10)

        # Socket UDP
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", port))
        self.get_logger().info(f"Escuchando comandos UDP en 0.0.0.0:{port}")

        # Watchdog para parar si dejan de llegar comandos
        self.timeout_seconds = 0.5  # si no llega nada en 0.5 s -> detener
        self.last_msg_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.watchdog_callback)

        # Hilo para recibir UDP
        self.running = True
        self.thread = threading.Thread(target=self.udp_loop, daemon=True)
        self.thread.start()

    def udp_loop(self):
        self.get_logger().info("Hilo UDP iniciado.")
        while self.running:
            try:
                data, addr = self.sock.recvfrom(1024)  # bloqueante
                text = data.decode("utf-8").strip()
                # Esperamos formato "vx,wz"
                try:
                    vx_str, wz_str = text.split(",")
                    vx = float(vx_str)
                    wz = float(wz_str)

                    twist = Twist()
                    twist.linear.x = vx
                    twist.angular.z = wz
                    self.pub_cmd_vel.publish(twist)

                    self.last_msg_time = self.get_clock().now()
                except ValueError:
                    self.get_logger().warn(f"Mensaje inválido desde {addr}: '{text}'")
            except Exception as e:
                self.get_logger().error(f"Error en udp_loop: {e}")
                break

        self.get_logger().info("Hilo UDP finalizado.")

    def watchdog_callback(self):
        # Si pasa demasiado tiempo sin comandos, parar el robot
        now = self.get_clock().now()
        elapsed = (now - self.last_msg_time).nanoseconds / 1e9
        if elapsed > self.timeout_seconds:
            twist = Twist()  # todo en 0
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
