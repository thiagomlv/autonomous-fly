import cv2
import numpy as np
from time import sleep
from identificar_circulo import identificar_circulos_amarelos

# Capturar vídeo da webcam
cap = cv2.VideoCapture(0)

while True:
    # Ler um frame da webcam
    ret, frame = cap.read()
    
    if not ret:
        break
    
    # Identificar círculos amarelos no frame
    imagem, circulo, direcao = identificar_circulos_amarelos(frame)

    # Mostrar o frame original e o resultado
    cv2.imshow('Círculos Amarelos Identificados', imagem)

    # centraliza o circulo na imagem
    if circulo is not False:
        if direcao == "Acima":
            print("Enviar comando pra ir pra frente") # Envia o comando
            #sleep(5) # espera alguns instantes antes de verificar o próximo frame
        if direcao == "Abaixo":
            print("Enviar comando pra ir pra trás") # Envia o comando
            #sleep(5) # espera alguns instantes antes de verificar o próximo frame
        if direcao == "Direita":
            print("Enviar comando pra ir pra a direita") # Envia o comando
            #sleep(5) # espera alguns instantes antes de verificar o próximo frame
        if direcao == "Esquerda":
            print("Enviar comando pra ir para a esquerda") # Envia o comando
            #sleep(5) # espera alguns instantes antes de verificar o próximo frame
        if direcao == "Centro":
            break
    
    # Sair do loop quando a tecla 'q' for pressionada
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Liberar o objeto de captura e fechar todas as janelas
cap.release()
cv2.destroyAllWindows()
