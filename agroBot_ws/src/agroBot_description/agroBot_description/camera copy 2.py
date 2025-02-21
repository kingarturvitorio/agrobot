import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32  # Para publicar o número de alfaces detectados
from cv_bridge import CvBridge
import cv2
import numpy as np

class AlfaceDetector(Node):
    def __init__(self):
        super().__init__('alface_detector')
        self.bridge = CvBridge()

        # Assina a câmera
        self.subscription = self.create_subscription(
            Image, '/roboagricola_camera/image_raw', self.image_callback, 10)

        # Publica a imagem processada com as detecções
        self.publisher = self.create_publisher(Image, '/alface/detection', 10)

        # Publica o número de alfaces detectados
        self.counter_publisher = self.create_publisher(Int32, '/alface/count', 10)

    def image_callback(self, msg):
        # Converter a imagem ROS para OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Aplicar um filtro de cor para detectar verde (alface)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_green = np.array([35, 40, 40])   # Faixa de verde
        upper_green = np.array([85, 255, 255])
        mask = cv2.inRange(hsv, lower_green, upper_green)

        # Encontrar contornos da planta na máscara
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Contar quantas plantas de alface foram detectadas
        alface_count = 0
        for cnt in contours:
            if cv2.contourArea(cnt) > 500:  # Filtra pequenos ruídos
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                alface_count += 1  # Incrementa o contador para cada alface detectado
        
        # Publicar a imagem processada
        detection_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        self.publisher.publish(detection_msg)

        # Publicar o número de alfaces detectados
        count_msg = Int32()
        count_msg.data = alface_count
        self.counter_publisher.publish(count_msg)

        # Mostrar a imagem com detecção e exibir contador
        cv2.putText(frame, f"Alfaces detectados: {alface_count}", (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow('Detecção de Alface', frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = AlfaceDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
