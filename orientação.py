import pymavlink.mavutil as utility
import pymavlink.dialects.v20.all as dialect

# Função para receber e processar dados da IMU
def imu_orientacao():
    while True:
        # Receber mensagem IMU
        message = vehicle.recv_match(type='ATTITUDE', blocking=True)

        # Se a mensagem for válida
        if message:
            # Converter a mensagem para dicionário
            dados_imu = message.to_dict()

            # Exibir os dados da IMU
            roll, pitch, yaw = dados_imu['roll'],dados_imu['pitch'],dados_imu['yaw']

            # Roll (Rolo) indica o ângulo de rotação do drone em torno do eixo que conecta a frente e a traseira dele

            # Pitch (Arfagem) indica o ângulo de inclinação do drone para frente ou para trás

            # Yaw (Guinada) indica o ângulo de rotação do drone em relação ao eixo vertical, ou seja, para qual direção a frente do drone está apontando
            
            return roll, pitch, yaw
