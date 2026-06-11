"""maze_controller controller."""

from controller import Robot
import math
from path_planner import preprocess_maze, a_star 

def nodo_a_coordenada(nodo, tamano_celda=0.1):
    fila, columna = nodo
    
    x_fisico = (columna * tamano_celda) + 0.05
    y_fisico = ((30 - fila) * tamano_celda) + 0.05 
    
    return (x_fisico, y_fisico)

def normalizar_angulo(angulo):
    while angulo > math.pi:
        angulo -= 2.0 * math.pi
    while angulo < -math.pi:
        angulo += 2.0 * math.pi
    return angulo

def run_robot(robot):
    timestep = int(robot.getBasicTimeStep())
    
    # Config del Robot
    RADIO_RUEDA = 0.0205 
    DISTANCIA_RUEDAS = 0.052
    VELOCIDAD_MAXIMA = 6.28
    
    left_motor = robot.getDevice("left wheel motor")
    right_motor = robot.getDevice("right wheel motor")
    left_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    right_motor.setPosition(float('inf'))
    right_motor.setVelocity(0.0)
    
    # Habilitar Encoders
    left_position_sensor = robot.getDevice("left wheel sensor")
    right_position_sensor = robot.getDevice("right wheel sensor")
    left_position_sensor.enable(timestep)
    right_position_sensor.enable(timestep)
    
    last_ps_left = 0.0
    last_ps_right = 0.0
    
    # Hacer avanzar el simulador un paso en blanco para que 
    # todos los sensores se enciendan y llenen de datos reales
    robot.step(timestep)

    # Habilitar Sensores de proximidad
    prox_sensors = []
    for ind in range(8):
        sensor_name = 'ps' + str(ind)
        prox_sensors.append(robot.getDevice(sensor_name))
        prox_sensors[ind].enable(timestep)
    
    robot_x = 0.15
    robot_y = 0.15
    robot_phi = 1.5

    # Planificación Global
    maze_raw = [
        [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
        [1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1,'G',0,0,0,0,0,0,0,0,0,0,1],
        [1,0,1,0,1,0,1,1,1,0,1,1,1,1,1,1,1,0,1,1,1,1,1,0,1,1,1,1,1,0,1],
        [1,0,1,0,1,0,0,0,1,0,1,0,0,0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,1,0,1],
        [1,0,1,0,1,1,1,0,1,0,1,0,1,0,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,0,1],
        [1,0,1,0,1,0,0,0,1,0,0,0,1,0,0,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1],
        [1,0,1,1,1,0,1,1,1,1,1,0,1,1,1,1,1,0,1,0,1,1,1,0,1,0,1,0,1,0,1],
        [1,0,0,0,0,0,1,0,0,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,1],
        [1,1,1,1,1,1,1,0,1,1,1,1,1,0,1,0,1,0,1,1,1,0,1,0,1,1,1,1,1,0,1],
        [1,0,0,0,0,0,0,0,1,0,0,0,0,0,1,0,1,0,1,0,0,0,1,0,1,0,0,0,0,0,1],
        [1,0,1,1,1,1,1,0,1,0,1,1,1,1,1,0,1,0,1,1,1,0,1,0,1,0,1,1,1,1,1],
        [1,0,1,0,0,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,1,0,1,0,1,0,0,0,1],
        [1,0,1,0,1,1,1,1,1,1,1,0,1,0,1,1,1,0,1,0,1,1,1,0,1,0,1,1,1,0,1],
        [1,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,1,0,1,0,0,0,1,0,1,0,0,0,1,0,1],
        [1,0,1,1,1,0,1,0,1,1,1,1,1,1,1,0,1,0,1,1,1,0,1,0,1,1,1,0,1,0,1],
        [1,0,1,0,0,0,1,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,1,0,0,0,1,0,1,0,1],
        [1,0,1,0,1,1,1,0,1,0,1,0,1,0,1,1,1,0,1,0,1,0,1,1,1,0,1,0,1,0,1],
        [1,0,0,0,0,0,0,0,1,0,1,0,0,0,1,0,0,0,1,0,1,0,0,0,1,0,1,0,0,0,1],
        [1,0,1,1,1,1,1,1,1,0,1,1,1,1,1,0,1,0,1,0,1,1,1,0,1,0,1,1,1,0,1],
        [1,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,1,0,1,0,0,0,0,0,1,0,1,0,0,0,1],
        [1,0,1,0,1,0,1,1,1,1,1,0,1,1,1,1,1,0,1,1,1,0,1,1,1,0,1,0,1,1,1],
        [1,0,1,0,1,0,0,0,0,0,1,0,1,0,0,0,1,0,0,0,0,0,1,0,0,0,1,0,0,0,1],
        [1,0,1,0,1,1,1,1,1,0,1,0,1,0,1,0,1,1,1,1,1,1,1,1,1,0,1,0,1,0,1],
        [1,0,0,0,0,0,1,0,0,0,1,0,1,0,1,0,0,0,1,0,0,0,0,0,0,0,1,0,1,0,1],
        [1,1,1,1,1,1,1,0,1,1,1,0,1,0,1,1,1,0,1,0,1,0,1,1,1,1,1,0,1,0,1],
        [1,0,0,0,0,0,0,0,1,0,0,0,1,0,1,0,0,0,1,0,1,0,0,0,0,0,1,0,1,0,1],
        [1,0,1,1,1,1,1,1,1,0,1,1,1,0,1,0,1,1,1,1,1,1,1,0,1,0,1,0,1,0,1],
        [1,0,1,0,0,0,0,0,1,0,0,0,0,0,1,0,1,0,0,0,0,0,1,0,1,0,1,0,0,0,1],
        [1,0,1,0,1,1,1,0,1,1,1,1,1,1,1,0,1,0,1,1,1,0,1,1,1,0,1,1,1,0,1],
        [1,'S',1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1,0,0,0,1],
        [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1]
    ]
    
    print("Calculando ruta...")
    num_maze, start_node, goal_node = preprocess_maze(maze_raw)
    ruta_nodos = a_star(num_maze, start_node, goal_node)
    print(ruta_nodos)
    
    # Convertir nodos lógicos a coordenadas físicas (X, Y)
    ruta_fisica = [nodo_a_coordenada(nodo) for nodo in ruta_nodos]
    print(f"Ruta planificada con {len(ruta_fisica)} waypoints.")

    # Variables de Estado
    ESTADO_ROTANDO = 0
    ESTADO_AVANZANDO = 1
    ESTADO_EVADIENDO = 2
    estado_actual = ESTADO_ROTANDO
    indice_waypoint = 1
    
    TOLERANCIA_ANGULO = 0.08 
    TOLERANCIA_DISTANCIA = 0.05 
    
    # Umbrales
    UMBRAL_CHOQUE_FRONTAL = 400.0 # Solo si está a milímetros del frente
    UMBRAL_PARED_LATERAL = 150.0  # Para mantener centrado en el pasillo
    
    # Bucle Principal
    while robot.step(timestep) != -1:
        # ---------------
        # Odometría
        # ---------------
        current_ps_left = left_position_sensor.getValue()
        current_ps_right = right_position_sensor.getValue()
        
        delta_theta_l = current_ps_left - last_ps_left
        delta_theta_r = current_ps_right - last_ps_right
        
        delta_s_l = RADIO_RUEDA * delta_theta_l
        delta_s_r = RADIO_RUEDA * delta_theta_r
        
        delta_s = (delta_s_r + delta_s_l) / 2.0
        delta_phi = (delta_s_r - delta_s_l) / DISTANCIA_RUEDAS
        
        robot_x += delta_s * math.cos(robot_phi + (delta_phi / 2.0))
        robot_y += delta_s * math.sin(robot_phi + (delta_phi / 2.0))
        robot_phi += delta_phi
        robot_phi = normalizar_angulo(robot_phi)
        
        last_ps_left = current_ps_left
        last_ps_right = current_ps_right
        
        val_ps7 = prox_sensors[7].getValue()
        val_ps0 = prox_sensors[0].getValue()
        val_ps6 = prox_sensors[6].getValue()
        val_ps1 = prox_sensors[1].getValue()
        
        peligro_frontal = val_ps7 > UMBRAL_CHOQUE_FRONTAL or val_ps0 > UMBRAL_CHOQUE_FRONTAL
        pared_izquierda = val_ps6 > UMBRAL_PARED_LATERAL
        pared_derecha = val_ps1 > UMBRAL_PARED_LATERAL

        # ------------------
        # Toma de Decisiones
        # ------------------
        left_speed = 0.0
        right_speed = 0.0
        
        # Evitar choque de frente
        if peligro_frontal:
            estado_actual = ESTADO_EVADIENDO
            
        if estado_actual == ESTADO_EVADIENDO:
            # Escape rápido girando
            left_speed = -VELOCIDAD_MAXIMA * 0.5
            right_speed = VELOCIDAD_MAXIMA * 0.5
            
            if not peligro_frontal:
                estado_actual = ESTADO_ROTANDO
                
        # Seguir la ruta con corrección de pasillo
        elif indice_waypoint < len(ruta_fisica):
            objetivo_x, objetivo_y = ruta_fisica[indice_waypoint]
            
            error_x = objetivo_x - robot_x
            error_y = objetivo_y - robot_y
            
            distancia_al_objetivo = math.sqrt(error_x**2 + error_y**2)
            angulo_deseado = math.atan2(error_y, error_x)
            error_angular = normalizar_angulo(angulo_deseado - robot_phi)
            
            if estado_actual == ESTADO_ROTANDO:
                if abs(error_angular) > TOLERANCIA_ANGULO:
                    velocidad_giro = VELOCIDAD_MAXIMA * 0.2 
                    if error_angular > 0:
                        left_speed = -velocidad_giro
                        right_speed = velocidad_giro
                    else:
                        left_speed = velocidad_giro
                        right_speed = -velocidad_giro
                else:
                    estado_actual = ESTADO_AVANZANDO
                    
            elif estado_actual == ESTADO_AVANZANDO:
                if distancia_al_objetivo > TOLERANCIA_DISTANCIA:
                    kp = 2.0
                    ajuste_rumbo = error_angular * kp
                    
                    # Corrección lateral
                    correccion_pared = 0.0
                    if pared_izquierda:
                        correccion_pared = 1.0  # (gira a la derecha)
                    elif pared_derecha:
                        correccion_pared = -1.0 # (gira a la izquierda)
                    
                    velocidad_base = VELOCIDAD_MAXIMA * 0.4
                    
                    left_speed = velocidad_base - ajuste_rumbo + correccion_pared
                    right_speed = velocidad_base + ajuste_rumbo - correccion_pared
                else:
                    print(f"Waypoint {indice_waypoint} alcanzado. Coordenada lógica: {ruta_nodos[indice_waypoint]}")
                    indice_waypoint += 1
                    estado_actual = ESTADO_ROTANDO 
        else:
            print("Meta Alcanzada!")
            left_speed = 0.0
            right_speed = 0.0
            
        # ------------------
        # Accionamiento
        # ------------------
        left_speed = max(min(left_speed, VELOCIDAD_MAXIMA), -VELOCIDAD_MAXIMA)
        right_speed = max(min(right_speed, VELOCIDAD_MAXIMA), -VELOCIDAD_MAXIMA)
        
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)

if __name__ == "__main__":
    my_robot = Robot()
    run_robot(my_robot)