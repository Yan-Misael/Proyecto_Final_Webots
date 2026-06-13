"""medium_maze_controller controller"""

from controller import Robot
import math
from path_planner import preprocess_maze, a_star

# Position: 0.225 0.225 -6.53-05
# Rotation: 0 0 1 1.5

def nodo_a_coordenada(nodo, tamano_celda=0.15):
    fila, columna = nodo
    # El laberinto tiene 21 filas (índice 0..20), la fila 20 es la de abajo
    x_fisico = (columna * tamano_celda) + (tamano_celda / 2.0)
    y_fisico = ((20 - fila) * tamano_celda) + (tamano_celda / 2.0)
    return (x_fisico, y_fisico)

def normalizar_angulo(angulo):
    while angulo > math.pi:
        angulo -= 2.0 * math.pi
    while angulo < -math.pi:
        angulo += 2.0 * math.pi
    return angulo

def promedio(datos):
    return sum(datos) / len(datos)

def run_robot(robot):
    timestep = int(robot.getBasicTimeStep())

    RADIO_RUEDA = 0.02
    DISTANCIA_RUEDAS = 0.052
    VELOCIDAD_MAXIMA = 6.28

    left_motor = robot.getDevice("left wheel motor")
    right_motor = robot.getDevice("right wheel motor")
    left_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    right_motor.setPosition(float('inf'))
    right_motor.setVelocity(0.0)

    left_position_sensor = robot.getDevice("left wheel sensor")
    right_position_sensor = robot.getDevice("right wheel sensor")
    left_position_sensor.enable(timestep)
    right_position_sensor.enable(timestep)

    imu = robot.getDevice("inertial unit")
    imu.enable(timestep)

    lidar = robot.getDevice("lidar")
    lidar.enable(timestep)

    robot.step(timestep)

    prox_sensors = []
    for ind in range(8):
        sensor_name = 'ps' + str(ind)
        prox_sensors.append(robot.getDevice(sensor_name))
        prox_sensors[ind].enable(timestep)

    maze_raw = [
        [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
        [1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1],
        [1,0,1,0,1,1,1,0,1,0,1,1,1,1,1,0,1,1,1,0,1],
        [1,0,1,0,1,0,0,0,1,0,1,0,0,0,0,0,1,0,0,0,1],
        [1,1,1,0,1,0,1,1,1,0,1,0,1,1,1,1,1,1,1,0,1],
        [1,0,0,0,0,0,1,0,0,0,1,0,0,0,0,0,0,0,1,0,1],
        [1,0,1,0,1,1,1,0,1,1,1,1,1,1,1,1,1,0,1,0,1],
        [1,0,1,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,1,'G',1],
        [1,0,1,0,1,0,1,1,1,1,1,0,1,1,1,0,1,1,1,1,1],
        [1,0,1,0,1,0,0,0,0,0,1,0,1,0,0,0,1,0,0,0,1],
        [1,0,1,0,1,1,1,1,1,0,1,0,1,0,1,0,1,0,1,0,1],
        [1,0,1,0,1,0,0,0,1,0,1,0,1,0,0,0,1,0,1,0,1],
        [1,0,1,1,1,0,1,0,1,0,1,0,1,0,1,1,1,0,1,0,1],
        [1,0,0,0,0,0,1,0,0,0,0,0,1,0,0,0,1,0,1,0,1],
        [1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,0,1,0,1,0,1],
        [1,0,0,0,0,0,1,0,0,0,0,0,0,0,1,0,1,0,1,0,1],
        [1,0,1,1,1,0,1,1,1,1,1,0,1,0,1,0,1,0,1,0,1],
        [1,0,1,0,1,0,1,0,0,0,0,0,1,0,1,0,0,0,1,0,1],
        [1,0,1,0,1,0,1,0,1,0,1,1,1,0,1,1,1,1,1,0,1],
        [1,'S',1,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1],
        [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1]
    ]

    print("Calculando ruta...")
    num_maze, start_node, goal_node = preprocess_maze(maze_raw)
    ruta_nodos = a_star(num_maze, start_node, goal_node)
    print(f"Ruta de {len(ruta_nodos)} nodos \nNodos a recorrer: {ruta_nodos}")

    def simplificar_ruta(nodos):
        if len(nodos) <= 2:
            return nodos
        simplificada = [nodos[0]]
        for i in range(1, len(nodos) - 1):
            dr_antes = (nodos[i][0] - nodos[i-1][0], nodos[i][1] - nodos[i-1][1])
            dr_despues = (nodos[i+1][0] - nodos[i][0], nodos[i+1][1] - nodos[i][1])
            if dr_antes != dr_despues:
                simplificada.append(nodos[i])
        simplificada.append(nodos[-1])
        return simplificada

    ruta_nodos = simplificar_ruta(ruta_nodos)
    ruta_fisica = [nodo_a_coordenada(nodo) for nodo in ruta_nodos]
    print(f"Ruta simplificada: {len(ruta_fisica)} waypoints.")

    robot_x, robot_y = ruta_fisica[0]
    
    _, _, yaw = imu.getRollPitchYaw()
    robot_phi = yaw

    last_ps_left = left_position_sensor.getValue()
    last_ps_right = right_position_sensor.getValue()

    ESTADO_ROTANDO   = 0
    ESTADO_AVANZANDO = 1
    ESTADO_RECUPERANDO = 2
    estado_actual = ESTADO_ROTANDO
    indice_waypoint = 1

    TOLERANCIA_ANGULO = 0.03
    distancia_al_objetivo = float('inf')

    # Detección de atasco
    _pos_anterior_x = 0.0
    _pos_anterior_y = 0.0
    _ciclos_sin_avance = 0
    _CICLOS_ATASCO = 80        # ~2.5s sin moverse → recuperar
    _DIST_MIN_AVANCE = 0.002   # debe moverse al menos 2mm cada 80 ciclos

    # --- MÉTRICAS REQUERIDAS POR EL PROYECTO FINAL ---
    distancia_total_recorrida = 0.0
    conteo_riesgos_colision = 0
    tiempo_inicio = robot.getTime()
    metricas_mostradas = False

    while robot.step(timestep) != -1:

        # ---------- Odometría y Sensores ----------
        ranges = lidar.getRangeImage()
        dist_der    = promedio(ranges[0:20])
        dist_frente = promedio(ranges[246:266])
        dist_izq    = promedio(ranges[492:512])

        current_ps_left  = left_position_sensor.getValue()
        current_ps_right = right_position_sensor.getValue()

        delta_theta_l = current_ps_left  - last_ps_left
        delta_theta_r = current_ps_right - last_ps_right

        delta_s_l = RADIO_RUEDA * delta_theta_l
        delta_s_r = RADIO_RUEDA * delta_theta_r
        delta_s   = (delta_s_r + delta_s_l) / 2.0

        _, _, yaw = imu.getRollPitchYaw()
        robot_phi = yaw

        robot_x += delta_s * math.cos(robot_phi)
        robot_y += delta_s * math.sin(robot_phi)

        # Acumular distancia recorrida paso a paso
        distancia_total_recorrida += abs(delta_s)

        last_ps_left  = current_ps_left
        last_ps_right = current_ps_right

        # ---------- Control de Estados ----------
        left_speed = 0.0
        right_speed = 0.0
        correccion_lateral = 0.0

        if indice_waypoint < len(ruta_fisica):
            objetivo_x, objetivo_y = ruta_fisica[indice_waypoint]
            pasado_x, pasado_y = ruta_fisica[indice_waypoint - 1]

            error_x = objetivo_x - robot_x
            error_y = objetivo_y - robot_y

            distancia_al_objetivo = math.sqrt(error_x**2 + error_y**2)
            angulo_deseado  = math.atan2(error_y, error_x)
            error_angular   = normalizar_angulo(angulo_deseado - robot_phi)

            s_x = objetivo_x - pasado_x
            s_y = objetivo_y - pasado_y
            largo_tramo_cuadrado = s_x**2 + s_y**2

            r_x = robot_x - pasado_x
            r_y = robot_y - pasado_y

            producto_punto = r_x * s_x + r_y * s_y
            ha_cruzado_linea = producto_punto >= largo_tramo_cuadrado

            if s_x != 0:  
                distancia_lineal = abs(objetivo_x - robot_x)
            else:         
                distancia_lineal = abs(objetivo_y - robot_y)

            if estado_actual == ESTADO_ROTANDO:
                if abs(error_angular) > TOLERANCIA_ANGULO:
                    velocidad_giro = max(0.12, min(0.40, abs(error_angular) * 1.8)) * VELOCIDAD_MAXIMA
                    if error_angular > 0:
                        left_speed  = -velocidad_giro
                        right_speed =  velocidad_giro
                    else:
                        left_speed  =  velocidad_giro
                        right_speed = -velocidad_giro
                else:
                    estado_actual = ESTADO_AVANZANDO
                    _ciclos_sin_avance = 0
                    _pos_anterior_x = robot_x
                    _pos_anterior_y = robot_y

            elif estado_actual == ESTADO_RECUPERANDO:
                # Retroceder brevemente y reintentar la rotación
                left_speed  = -VELOCIDAD_MAXIMA * 0.30
                right_speed = -VELOCIDAD_MAXIMA * 0.30
                _ciclos_sin_avance += 1
                if _ciclos_sin_avance > 30:
                    print("Recuperación completada — reintentando waypoint")
                    estado_actual = ESTADO_ROTANDO
                    _ciclos_sin_avance = 0

            elif estado_actual == ESTADO_AVANZANDO:
                # Detección de atasco: verificar cada _CICLOS_ATASCO si avanzó suficiente
                _ciclos_sin_avance += 1
                if _ciclos_sin_avance >= _CICLOS_ATASCO:
                    dist_recorrida = math.sqrt(
                        (robot_x - _pos_anterior_x)**2 + (robot_y - _pos_anterior_y)**2
                    )
                    if dist_recorrida < _DIST_MIN_AVANCE:
                        print(f"ATASCO detectado en WP:{indice_waypoint} — recuperando")
                        estado_actual = ESTADO_RECUPERANDO
                        _ciclos_sin_avance = 0
                    else:
                        _pos_anterior_x = robot_x
                        _pos_anterior_y = robot_y
                        _ciclos_sin_avance = 0

                if not ha_cruzado_linea:
                    kp = 3.5
                    ajuste_rumbo = error_angular * kp

                    velocidad_base = min(
                        VELOCIDAD_MAXIMA * 0.60,
                        max(VELOCIDAD_MAXIMA * 0.08, distancia_lineal * 15)
                    )

                    # --- CORRECCIÓN LATERAL ---
                    # Con celdas de 0.15m: pasillo ~0.15m, centro a 0.075m de cada pared
                    UMBRAL_PASILLO = 0.12
                    PARED_CRITICA  = 0.07
                    PARED_ALERTA   = 0.09

                    if dist_izq < PARED_CRITICA or dist_der < PARED_CRITICA:
                        conteo_riesgos_colision += 1

                    hay_pared_izq = dist_izq < UMBRAL_PASILLO
                    hay_pared_der = dist_der < UMBRAL_PASILLO

                    if hay_pared_izq and hay_pared_der:
                        error_centro = dist_izq - dist_der
                        kp_pared = 2.0
                        correccion_lateral = max(
                            min(error_centro * kp_pared * VELOCIDAD_MAXIMA, VELOCIDAD_MAXIMA * 0.20),
                            -VELOCIDAD_MAXIMA * 0.20
                        )
                        if abs(error_centro) > 0.015:
                            velocidad_base *= 0.80

                    elif abs(error_angular) < 0.08:
                        if dist_izq < PARED_CRITICA:
                            velocidad_base     = VELOCIDAD_MAXIMA * 0.25
                            correccion_lateral = VELOCIDAD_MAXIMA * 0.30
                        elif dist_der < PARED_CRITICA:
                            velocidad_base     = VELOCIDAD_MAXIMA * 0.25
                            correccion_lateral = -VELOCIDAD_MAXIMA * 0.30
                        elif dist_izq < PARED_ALERTA:
                            correccion_lateral = VELOCIDAD_MAXIMA * 0.12
                        elif dist_der < PARED_ALERTA:
                            correccion_lateral = -VELOCIDAD_MAXIMA * 0.12

                    left_speed  = velocidad_base - ajuste_rumbo + correccion_lateral
                    right_speed = velocidad_base + ajuste_rumbo - correccion_lateral

                    print(f"WP:{indice_waypoint} dist:{distancia_lineal:.3f} "
                          f"IZQ:{dist_izq:.3f} DER:{dist_der:.3f} FRENTE:{dist_frente:.3f} "
                          f"ErrAng:{error_angular:.2f} CorrLat:{correccion_lateral:.3f} "
                          f"VBase:{velocidad_base:.2f} Pos:({robot_x:.3f},{robot_y:.3f})")
                else:
                    print(f"¡Intersección alcanzada! WP {indice_waypoint}")
                    robot_x = objetivo_x
                    robot_y = objetivo_y
                    indice_waypoint += 1
                    estado_actual = ESTADO_ROTANDO
                    _ciclos_sin_avance = 0
                    _pos_anterior_x = robot_x
                    _pos_anterior_y = robot_y
        else:
            left_speed  = 0.0
            right_speed = 0.0
            if not metricas_mostradas:
                tiempo_total = robot.getTime() - tiempo_inicio
                print("\n=============================================")
                print("¡MÁXIMA META ALCANZADA CON ÉXITO!")
                print(f" Tiempo Total de Ejecución: {tiempo_total:.2f} s")
                print(f" Distancia Total Recorrida: {distancia_total_recorrida:.3f} m")
                print(f" Instantes de Riesgo de Roce: {conteo_riesgos_colision}")
                print("=============================================\n")
                metricas_mostradas = True

        left_speed  = max(min(left_speed,  VELOCIDAD_MAXIMA), -VELOCIDAD_MAXIMA)
        right_speed = max(min(right_speed, VELOCIDAD_MAXIMA), -VELOCIDAD_MAXIMA)

        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)

if __name__ == "__main__":
    my_robot = Robot()
    run_robot(my_robot)
