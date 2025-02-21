import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32  # Para publicar a contagem total de alfaces
from cv_bridge import CvBridge
import cv2
import numpy as np

class AlfaceDetector(Node):
    def __init__(self):
        super().__init__('alface_detector')
        self.bridge = CvBridge()

        # Contador global de alfaces detectados
        self.total_alface_count = 0
        self.detected_lettuces = []  # Lista para armazenar posições dos alfaces detectados
        self.detection_threshold = 50  # Distância mínima para considerar um alface novo

        # Assina a câmera
        self.subscription = self.create_subscription(
            Image, '/roboagricola_camera/image_raw', self.image_callback, 10)

        # Publica a imagem processada
        self.publisher = self.create_publisher(Image, '/alface/detection', 10)

        # Publica o número acumulado de alfaces detectados
        self.counter_publisher = self.create_publisher(Int32, '/alface/total_count', 10)

    def is_new_detection(self, x, y, w, h):
        """
        Verifica se a nova detecção é realmente um novo alface,
        comparando com os já detectados anteriormente.
        """
        for (px, py, pw, ph) in self.detected_lettuces:
            if abs(x - px) < self.detection_threshold and abs(y - py) < self.detection_threshold:
                return False  # Se a nova detecção está muito perto de uma antiga, ignora
        return True

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
        
        for cnt in contours:
            if cv2.contourArea(cnt) > 500:  # Filtra pequenos ruídos
                x, y, w, h = cv2.boundingRect(cnt)

                # Verifica se essa detecção já foi registrada antes
                if self.is_new_detection(x, y, w, h):
                    self.total_alface_count += 1
                    self.detected_lettuces.append((x, y, w, h))  # Salva a nova detecção
                    self.get_logger().info(f"Novo alface detectado! Total: {self.total_alface_count}")

                # Desenha o retângulo na imagem
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)

        # Publicar a imagem processada
        detection_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        self.publisher.publish(detection_msg)

        # Publicar o total de alfaces detectados
        count_msg = Int32()
        count_msg.data = self.total_alface_count
        self.counter_publisher.publish(count_msg)

        # Mostrar a imagem com detecção e exibir contador acumulado
        cv2.putText(frame, f"Total de alfaces: {self.total_alface_count}", (10, 50),
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
