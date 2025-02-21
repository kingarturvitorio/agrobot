import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class AlfaceDetector(Node):
    def __init__(self):
        super().__init__('alface_detector')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image, '/roboagricola_camera/image_raw', self.image_callback, 10)
        self.publisher = self.create_publisher(Image, '/alface/detection', 10)

    def image_callback(self, msg):
        # Converter a imagem ROS para OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Converter para HSV e filtrar a cor verde (alface)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_green = np.array([35, 40, 40])
        upper_green = np.array([85, 255, 255])
        mask = cv2.inRange(hsv, lower_green, upper_green)

        # Encontrar contornos da planta na máscara
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            if cv2.contourArea(cnt) > 500:  # Filtrar pequenos objetos
                x, y, w, h = cv2.boundingRect(cnt)

                # Expandir a área ao redor do alface (Zona de Ação)
                padding = 30  # Aumenta o retângulo ao redor do alface
                x1, y1 = max(0, x - padding), max(0, y - padding)
                x2, y2 = min(frame.shape[1], x + w + padding), min(frame.shape[0], y + h + padding)

                # Criar uma máscara para a área expandida
                overlay = frame.copy()
                cv2.rectangle(overlay, (x1, y1), (x2, y2), (0, 0, 255), -1)  # Preenchido com vermelho
                alpha = 0.3  # Transparência
                frame = cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0)

                # Destacar o alface dentro da zona de ação
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)  # Verde para o alface
                cv2.putText(frame, "Zona de Remocao", (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # Publicar a imagem com a zona de ação desenhada
        detection_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        self.publisher.publish(detection_msg)

        # Exibir a imagem para debug
        cv2.imshow('Detecção de Alface com Zona de Ação', frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = AlfaceDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
