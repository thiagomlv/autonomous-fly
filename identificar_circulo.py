import cv2
import numpy as np
from time import sleep

# Função para identificar círculos amarelos em um frame
def identificar_circulos_amarelos(frame):
    """
    Recebe O frame a ser analisado
    Caso encontre um círculo no frame:
         Retorna: o frame com o desenho do circulo, uma tupla com (x, y, raio), a direção de onde está o circulo em relação ao centro da imagem
    Caso não encontre um círculo:
        Retorna: o frame, False , False
    """

    # Determinar as dimensões da imagem
    altura, largura, _ = frame.shape
    
    # Determinar o centro da imagem
    centro_imagem_x = largura // 2
    centro_imagem_y = altura // 2

    cv2.rectangle(frame, (centro_imagem_x - 20, centro_imagem_y - 20), (centro_imagem_x + 20, centro_imagem_y + 20), (0, 255, 0), -1)

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
                                param1=50, param2=30, minRadius=60, maxRadius=150)
    
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
            if y < centro_imagem_y - 20:
                direcao = "Acima"
            elif y > centro_imagem_y + 20:
                direcao = "Abaixo"
            elif x < centro_imagem_x - 20:
                direcao = "Esquerda"
            elif x > centro_imagem_x + 20:
                direcao = "Direita"
            else:
                direcao = "Centro"

            return frame, maior_circulo, direcao
    
    return frame, False, False