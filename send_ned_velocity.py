import pymavlink.mavutil as utility
import pymavlink.dialects.v20.all as dialect
from time import sleep

def send_ned_velocity(drone, velocity_x, velocity_y, velocity_z, duration):
    """
    Move o drone em uma direção específica usando coordenadas NED (Norte, Leste, Baixo).

    :param velocity_x: Velocidade ao longo do eixo X (Norte) em m/s.
    :param velocity_y: Velocidade ao longo do eixo Y (Leste) em m/s.
    :param velocity_z: Velocidade ao longo do eixo Z (Baixo) em m/s.
    :param duration: Duração do movimento em segundos.
    """
    # Defina a taxa de envio de mensagens
    msg_rate = 10  # 10 Hz
    time_interval = 1.0 / msg_rate

    # Envie mensagens de posição NED em um loop
    for _ in range(int(duration * msg_rate)):
        # Crie a mensagem SET_POSITION_TARGET_LOCAL_NED
        msg = drone.mav.set_position_target_local_ned_encode(
            0,       # tempo_boot_ms (não usado)
            1,       # target system
            1,       # target component
            utility.mavlink.MAV_FRAME_LOCAL_NED,  # Frame de referência
            0b0000111111000111,  # Tipos de máscara para ignorar posição e aceleração
            0, 0, 0,  # Coordenadas NED de posição (ignoradas devido à máscara)
            velocity_x, velocity_y, velocity_z,  # Velocidade NED
            0, 0, 0,  # Aceleração NED (ignorada devido à máscara)
            0, 0  # yaw, yaw_rate (ignorado devido à máscara)
        )
        # Envie a mensagem ao drone
        drone.send_mavlink(msg)
        sleep(time_interval)

# Exemplo de uso: mover 2 metros para frente
velocity_x = 1  # m/s (frente)
velocity_y = 0  # m/s
velocity_z = 0  # m/s (nivelado)
duration = 2  # segundos