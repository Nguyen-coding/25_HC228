#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import socket
import serial
import threading


class UdpUartBridge(Node):
    def __init__(self, usb_port_num: int):
        super().__init__('udp_uart_bridge')
        self.udp_ip = self.get_ip()
        self.udp_port = 8888
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.udp_ip, self.udp_port))

        # --- USB 포트 설정 부분 ---
        uart_port = f'/dev/ttyUSB{usb_port_num}'
        try:
            self.ser = serial.Serial(uart_port, 115200, timeout=1)
            self.get_logger().info(f"UART 연결 성공: {uart_port}")
        except Exception as e:
            self.get_logger().error(f"UART 연결 실패 ({uart_port}): {e}")
            self.ser = None

        # --- UDP 수신 스레드 시작 ---
        self.udp_thread = threading.Thread(target=self.receive_udp_loop)
        self.udp_thread.daemon = True
        self.udp_thread.start()

        self.get_logger().info(f"UDP 수신 중: {self.udp_ip}:{self.udp_port}")

    def get_ip(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip

    def receive_udp_loop(self):
        while True:
            data, addr = self.sock.recvfrom(1024)
            msg = data.decode('utf-8')
            self.get_logger().info(f"UDP 수신: {msg} from {addr}")
            self.sock.sendto("Connected successfully!".encode('utf-8'), addr)

            if self.ser:
                try:
                    self.ser.reset_input_buffer()
                    self.ser.reset_output_buffer()
                    self.ser.write(data)
                    self.get_logger().info(f"UART 송신: {msg.strip()}")
                except Exception as e:
                    self.get_logger().error(f"UART 오류: {e}")
            else:
                self.get_logger().info(f"[디버그] UART 없음 - 메시지 출력만: {msg.strip()}")


def main(args=None):
    rclpy.init(args=args)

    # --- 실행 시 포트 번호 입력 ---
    try:
        port_num = int(input("연결할 USB 포트 번호 입력 (예: 0 → /dev/ttyUSB0): "))
    except ValueError:
        print("잘못된 입력입니다. 기본 포트(0)로 설정합니다.")
        port_num = 0

    node = UdpUartBridge(port_num)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
