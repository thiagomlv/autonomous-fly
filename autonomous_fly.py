import time
import pymavlink.mavutil as utility
import pymavlink.dialects.v20.all as dialect
import cv2
import numpy as np

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

# Constants
TAKEOFF_ALTITUDE = 2
FORWARD_SPEED = 0.5  # Speed in m/s (adjust as needed)
SLEEP_INTERVAL = 0.1  # Interval to send commands

# Connect to vehicle
vehicle = utility.mavlink_connection(device="udpin:127.0.0.1:14550")

# Wait for a heartbeat
vehicle.wait_heartbeat()

# Inform user
print("Connected to system:", vehicle.target_system, ", component:", vehicle.target_component)

# Capturar vídeo da webcam
cap = cv2.VideoCapture(0)

# Ensure the vehicle is in GUIDED mode
print("Setting mode to GUIDED")
vehicle.mav.command_long_send(
    vehicle.target_system,
    vehicle.target_component,
    dialect.MAV_CMD_DO_SET_MODE,
    0,
    1,  # Base mode: GUIDED
    4,  # Custom mode
    0, 0, 0, 0, 0
)
time.sleep(2)  # Give some time to change mode

# Create takeoff command
takeoff_command = dialect.MAVLink_command_long_message(
    target_system=vehicle.target_system,
    target_component=vehicle.target_component,
    command=dialect.MAV_CMD_NAV_TAKEOFF,
    confirmation=0,
    param1=0,
    param2=0,
    param3=0,
    param4=0,
    param5=0,
    param6=0,
    param7=TAKEOFF_ALTITUDE
)

# Create land command
land_command = dialect.MAVLink_command_long_message(
    target_system=vehicle.target_system,
    target_component=vehicle.target_component,
    command=dialect.MAV_CMD_NAV_LAND,
    confirmation=0,
    param1=0,
    param2=0,
    param3=0,
    param4=0,
    param5=0,
    param6=0,
    param7=0
)

# Takeoff the vehicle
vehicle.mav.send(takeoff_command)
print("Sent takeoff command to vehicle")

# Check if takeoff is successful
while True:
    # Ler um frame da webcam
    ret, frame = cap.read()
    if not ret:
        break
    # Identificar círculos amarelos no frame
    imagem, circulo, direcao = identificar_circulos_amarelos(frame)

    # Mostrar o frame original e o resultado
    cv2.imshow('Webcam', imagem)

    message = vehicle.recv_match(type=dialect.MAVLink_global_position_int_message.msgname, blocking=True).to_dict()
    relative_altitude = message["relative_alt"] * 1e-3
    print("Relative Altitude", relative_altitude, "meters")
    if TAKEOFF_ALTITUDE - relative_altitude < 1:
        print("Takeoff to", TAKEOFF_ALTITUDE, "meters is successful")
        break


print(f"Moving forward and looking for the platform")
while True:
    # Ler um frame da webcam
    ret, frame = cap.read()
    if not ret:
        break
    # Identificar círculos amarelos no frame
    imagem, circulo, direcao = identificar_circulos_amarelos(frame)
    
    # Mostrar o frame original e o resultado
    cv2.imshow('Webcam', imagem)

    if circulo is not False:
        break

    forward_command = dialect.MAVLink_set_position_target_local_ned_message(
        time_boot_ms=0,
        target_system=vehicle.target_system,
        target_component=vehicle.target_component,
        coordinate_frame=dialect.MAV_FRAME_LOCAL_NED,
        type_mask=0b0000111111000111,  # Ignore position, only use velocity components
        x=0.0, y=0.0, z=0.0,
        vx=float(FORWARD_SPEED), vy=0.0, vz=0.0,
        afx=0.0, afy=0.0, afz=0.0,
        yaw=0.0, yaw_rate=0.0
    )
    vehicle.mav.send(forward_command)
    time.sleep(SLEEP_INTERVAL)

# Stop the movement
stop_command = dialect.MAVLink_set_position_target_local_ned_message(
    time_boot_ms=0,
    target_system=vehicle.target_system,
    target_component=vehicle.target_component,
    coordinate_frame=dialect.MAV_FRAME_LOCAL_NED,
    type_mask=0b111111111000,  # Stop movement
    x=0, y=0, z=0,
    vx=0, vy=0, vz=0,
    afx=0, afy=0, afz=0,
    yaw=0, yaw_rate=0
)

vehicle.mav.send(stop_command)
print("Platform found, stopped forward movement")

print("Centralizando a plataforma")
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

            # ENVIAR AQUI O COMANDO

            time.sleep(5) # espera alguns instantes antes de verificar o próximo frame
        if direcao == "Abaixo":
            print("Enviar comando pra ir pra trás") # Envia o comando
            
            # ENVIAR AQUI O COMANDO
            
            time.sleep(5) # espera alguns instantes antes de verificar o próximo frame
        if direcao == "Direita":
            print("Enviar comando pra ir pra a direita") # Envia o comando

            # ENVIAR AQUI O COMANDO

            time.timesleep(5) # espera alguns instantes antes de verificar o próximo frame
        if direcao == "Esquerda":
            print("Enviar comando pra ir para a esquerda") # Envia o comando

            # ENVIAR AQUI O COMANDO

            time.sleep(5) # espera alguns instantes antes de verificar o próximo frame
        if direcao == "Centro":
            print("Plataforma Centralizada com sucesso!")
            break



# Land the vehicle
vehicle.mav.send(land_command)
print("Sent land command to vehicle")

# Check if landing is successful
while True:
    message = vehicle.recv_match(type=dialect.MAVLink_global_position_int_message.msgname, blocking=True).to_dict()
    relative_altitude = message["relative_alt"] * 1e-3
    print("Relative Altitude", relative_altitude, "meters")
    if relative_altitude < 1:
        break

# Wait some seconds to ensure landing
time.sleep(5)
print("Landed successfully")
cap.release()
cv2.destroyAllWindows()
