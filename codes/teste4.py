"""
Testa somente a câmera, sem mover o drone.
"""

import cv2
import numpy as np
from time import sleep
import os

# Função para identificar círculos amarelos em um frame
def identificar_circulos_amarelos(frame):

    # Determinar as dimensões da imagem
    altura, largura, _ = frame.shape
    
    # Determinar o centro da imagem
    centro_imagem_x = largura // 2
    centro_imagem_y = altura // 2

    # Converter a imagem para o espaço de cores HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Definir o intervalo para a cor amarela
    amarelo_claro = np.array([20, 100, 100])
    amarelo_escuro = np.array([35, 255, 255])
    
    # Criar uma máscara que captura apenas os pixels amarelos
    mascara = cv2.inRange(hsv, amarelo_claro, amarelo_escuro)
    
    # Aplicar uma série de operações morfológicas para reduzir o ruído
    kernel = np.ones((5, 5), np.uint8)
    mascara = cv2.morphologyEx(mascara, cv2.MORPH_CLOSE, kernel)
    mascara = cv2.morphologyEx(mascara, cv2.MORPH_OPEN, kernel)
    cv2.imshow('Mascara', mascara)
    
    # Detectar círculos usando a Transformada de Hough
    circulos = cv2.HoughCircles(mascara, cv2.HOUGH_GRADIENT, dp=1.2, minDist=50,
                                param1=50, param2=30, minRadius=100, maxRadius=150)
    
    # Se algum círculo foi detectado, desenhar apenas o círculo com o maior raio
    if circulos is not None:
        circulos = np.round(circulos[0, :]).astype("int")
        maior_raio = 0
        maior_circulo = None
        for (x, y, r) in circulos:
            if r > maior_raio:  # Verificar se o raio do círculo é maior ou igual ao mínimo
                maior_raio = r
                maior_circulo = (x, y, r)
        
        # Desenhar o círculo com o maior raio encontrado
        if maior_circulo is not None:
            x, y, r = maior_circulo
            cv2.circle(frame, (x, y), r, (0, 255, 0), 4)
            cv2.rectangle(frame, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
            
            # Determinar a posição do centro do círculo em relação ao centro da imagem
            if y > centro_imagem_y + 20:
                direcao = "Acima"
            elif y < centro_imagem_y - 20:
                direcao = "Abaixo"
            elif x < centro_imagem_x - 20:
                direcao = "Esquerda"
            elif x > centro_imagem_x + 20:
                direcao = "Direita"
            else:
                direcao = "Centro"

            return frame, maior_circulo, direcao
    
    return frame, False, False

# Capturar vídeo da webcam
cap = cv2.VideoCapture(0)
H = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
W = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))

# path = 'set/output/path/to/the/recording'
# outputFolder = os.path.join(path, 'recordingHarpia.MP4')
# out = cv2.VideoWriter(outputFolder, cv2.VideoWriter_fourcc(*'XVID'), int(cap.get(cv2.CAP_PROP_FPS)), (W, H))

while True:
    # Ler um frame da webcam
    ret, frame = cap.read()
    
    if not ret:
        break
    
    # Identificar círculos amarelos no frame
    imagem, circulo, direcao = identificar_circulos_amarelos(frame)
    
    # Sair do loop quando a tecla 'q' for pressionada
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Liberar o objeto de captura e fechar todas as janelas
cap.release()
cv2.destroyAllWindows()
