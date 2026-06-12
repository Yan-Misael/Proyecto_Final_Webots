"""medium_maze_controller controller"""

from controller import Robot
import math
from path_planner import preprocess_maze, simplificar_ruta, a_star

# Position: 0.225 0.225 -6.53-05
# Rotation: 0 0 1 1.5

# =========================
#  PARÁMETROS CONFIGURABLES
# =========================
                    
# Geometría del robot
RADIO_RUEDA      = 0.02
DISTANCIA_RUEDAS = 0.052
VELOCIDAD_MAXIMA = 6.28

# Tolerancias de llegada
TOLERANCIA_ANGULO    = 0.03
TOLERANCIA_DISTANCIA = 0.01
FACTOR_SOBREPASO     = 5.0

# Control de giro (ESTADO_ROTANDO)
KP_GIRO       = 3.0
FRAC_GIRO_MAX = 0.30
FRAC_GIRO_MIN = 0.06

# Control de avance (ESTADO_AVANZANDO)
KP_RUMBO         = 3.0
FRAC_AVANCE      = 0.5
DIST_FRENADO     = 0.2
FRAC_FRENADO_MAX = 0.38
FRAC_FRENADO_MIN = 0.08

# Control lateral
KP_MUROS          = 0.005
UMBRAL_PARED_LIBRE = 75.0
DIST_APAGADO_LAT   = 0.07

# Control Muro
UMBRAL_PELIGRO_LATERAL = 100.0
KP_MUROS_NUEVO = 0.010

# Detección de obstáculos
UMBRAL_CHOQUE_FRONTAL = 400.0
UMBRAL_PARED_LATERAL  = 150.0

# Recuperación por evasión repetida
MAX_EVASIONES_WP = 3

# ================================================================

def nodo_a_coordenada(nodo, tamano_celda=0.15):
    fila, columna = nodo
    x_fisico = (columna * tamano_celda) + 0.075
    y_fisico = ((20 - fila) * tamano_celda) + 0.075 
    
    return (x_fisico, y_fisico)
    
def normalizar_angulo(angulo):
    while angulo > math.pi:
        angulo -= 2.0 * math.pi
    while angulo < -math.pi:
        angulo += 2.0 * math.pi
    return angulo


def run_robot(robot):
    timestep = int(robot.getBasicTimeStep())

    # Actuadores
    left_motor  = robot.getDevice("left wheel motor")
    right_motor = robot.getDevice("right wheel motor")
    left_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    right_motor.setPosition(float('inf'))
    right_motor.setVelocity(0.0)

    # Encoders
    left_sensor  = robot.getDevice("left wheel sensor")
    right_sensor = robot.getDevice("right wheel sensor")
    left_sensor.enable(timestep)
    right_sensor.enable(timestep)
    last_left  = 0.0
    last_right = 0.0

    robot.step(timestep)

    # Sensores de proximidad
    prox = []
    for i in range(8):
        s = robot.getDevice('ps' + str(i))
        s.enable(timestep)
        prox.append(s)
        
    # Pose inicial
    robot_x   = 0.3
    robot_y   = 0.3
    robot_phi = math.pi / 2.0

    # Planificación global
    #"""
    maze_raw = [
    
        [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
        [1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1],
        [1,0,1,0,1,1,1,0,1,0,1,1,1,1,1,0,1,1,1,0,1],
        [1,0,1,0,1,0,0,0,1,0,1,0,0,0,0,0,1,0,0,0,1],
        [1,1,1,0,1,0,1,1,1,0,1,0,1,1,1,1,1,1,1,0,1],
        [1,0,0,0,0,0,1,0,0,0,1,0,0,0,0,0,0,0,1,0,1],
        [1,0,1,0,1,1,1,0,1,1,1,1,1,1,1,1,1,0,1,0,1],
        [1,0,1,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,1,0,1],
        [1,0,1,0,1,0,1,1,1,1,1,0,1,1,1,0,1,1,1,1,1],
        [1,0,1,0,1,0,0,0,0,0,1,0,1,0,0,0,1,0,0,'G',1],
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
    """
    maze_raw = [
        [
        ]
    ]
    """

    print("Calculando ruta...")
    num_maze, start_node, goal_node = preprocess_maze(maze_raw)
    ruta_densa  = a_star(num_maze, start_node, goal_node)
    ruta_nodos  = simplificar_ruta(ruta_densa)
    ruta_fisica = [nodo_a_coordenada(n) for n in ruta_nodos]
    print(f"Ruta simplificada a {len(ruta_nodos)} esquinas clave: {ruta_nodos}")

    # Variables de estado
    ESTADO_ROTANDO   = 0
    ESTADO_AVANZANDO = 1
    ESTADO_EVADIENDO = 2

    estado_actual   = ESTADO_ROTANDO
    indice_waypoint = 1
    evasiones_wp    = 0
    velocidad_lineal_actual = 0.0

    # ================
    #  BUCLE PRINCIPAL
    # ================
    while robot.step(timestep) != -1:

        # Odometría diferencial
        cur_l = left_sensor.getValue()
        cur_r = right_sensor.getValue()

        d_sl  = RADIO_RUEDA * (cur_l - last_left)
        d_sr  = RADIO_RUEDA * (cur_r - last_right)
        d_s   = (d_sr + d_sl) / 2.0
        d_phi = (d_sr - d_sl) / DISTANCIA_RUEDAS

        robot_x   += d_s * math.cos(robot_phi + d_phi / 2.0)
        robot_y   += d_s * math.sin(robot_phi + d_phi / 2.0)
        robot_phi  = normalizar_angulo(robot_phi + d_phi)

        last_left  = cur_l
        last_right = cur_r

        # Lectura de sensores
        ps = [p.getValue() for p in prox]
        peligro_frontal = ps[7] > UMBRAL_CHOQUE_FRONTAL or ps[0] > UMBRAL_CHOQUE_FRONTAL
        pared_izquierda = ps[6] > UMBRAL_PARED_LATERAL
        pared_derecha   = ps[1] > UMBRAL_PARED_LATERAL

        # Decisión de velocidades
        left_speed  = 0.0
        right_speed = 0.0

        if peligro_frontal:
            estado_actual = ESTADO_EVADIENDO

        # ESTADO: EVADIENDO
        if estado_actual == ESTADO_EVADIENDO:
            left_speed  = -VELOCIDAD_MAXIMA * 0.5
            right_speed =  VELOCIDAD_MAXIMA * 0.5
            if not peligro_frontal:
                evasiones_wp += 1
                print(f"  [EVASION #{evasiones_wp}] WP {indice_waypoint} "
                      f"({ruta_nodos[indice_waypoint] if indice_waypoint < len(ruta_nodos) else 'META'})")
                estado_actual = ESTADO_ROTANDO

        # SEGUIMIENTO DE RUTA
        elif indice_waypoint < len(ruta_fisica):
            objetivo_x, objetivo_y = ruta_fisica[indice_waypoint]

            error_x = objetivo_x - robot_x
            error_y = objetivo_y - robot_y
            dist    = math.sqrt(error_x**2 + error_y**2)

            angulo_deseado = math.atan2(error_y, error_x)
            error_angular  = normalizar_angulo(angulo_deseado - robot_phi)

            # Detección de sobrepaso
            prod_esc       = error_x * math.cos(robot_phi) + error_y * math.sin(robot_phi)
            ha_sobrepasado = (prod_esc < 0) and (dist < TOLERANCIA_DISTANCIA * FACTOR_SOBREPASO)

            # Recuperación por evasiones repetidas
            if evasiones_wp >= MAX_EVASIONES_WP:
                print(f"  [SNAP FORZADO] WP {indice_waypoint} "
                      f"({ruta_nodos[indice_waypoint]}) — "
                      f"{evasiones_wp} evasiones. "
                      f"Odo: ({robot_x:.3f}, {robot_y:.3f}), "
                      f"Obj: ({objetivo_x:.3f}, {objetivo_y:.3f})")
                robot_x         = objetivo_x
                robot_y         = objetivo_y
                indice_waypoint += 1
                evasiones_wp    = 0
                estado_actual   = ESTADO_ROTANDO

            # ESTADO: ROTANDO
            elif estado_actual == ESTADO_ROTANDO:
                if abs(error_angular) > TOLERANCIA_ANGULO:
                    vel = abs(error_angular) * KP_GIRO
                    vel = max(min(vel, VELOCIDAD_MAXIMA * FRAC_GIRO_MAX),
                                      VELOCIDAD_MAXIMA * FRAC_GIRO_MIN)
                    if error_angular > 0:
                        left_speed, right_speed = -vel,  vel   # giro izquierda
                    else:
                        left_speed, right_speed =  vel, -vel   # giro derecha
                else:
                    robot_phi = round(robot_phi / (math.pi / 2.0)) * (math.pi / 2.0)
                    estado_actual = ESTADO_AVANZANDO
            
            
            # ESTADO: AVANZANDO
            elif estado_actual == ESTADO_AVANZANDO:

                alcanzado = (dist <= TOLERANCIA_DISTANCIA) or ha_sobrepasado

                if alcanzado:
                    if ha_sobrepasado:
                        print(f"  [SOBREPASO] WP {indice_waypoint} "
                              f"({ruta_nodos[indice_waypoint]}, dist={dist:.3f} m) "
                              f"— snap preventivo aplicado")
                    print(f"Waypoint {indice_waypoint} alcanzado. "
                          f"Coordenada lógica: {ruta_nodos[indice_waypoint]}")
                    robot_x         = objetivo_x
                    robot_y         = objetivo_y
                    evasiones_wp    = 0
                    velocidad_lineal_actual = 0.0 # Reiniciar inercia
                    indice_waypoint += 1
                    estado_actual   = ESTADO_ROTANDO
                    
                else:
                    ajuste_rumbo = error_angular * KP_RUMBO
                    
                    # Cálculo de la velocidad objetivo (Perfil de Frenado)
                    if dist >= DIST_FRENADO:
                        velocidad_objetivo = VELOCIDAD_MAXIMA * FRAC_AVANCE
                    else:
                        factor = dist / DIST_FRENADO
                        velocidad_objetivo = VELOCIDAD_MAXIMA * factor * FRAC_FRENADO_MAX
                        velocidad_objetivo = max(velocidad_objetivo,
                                                 VELOCIDAD_MAXIMA * FRAC_FRENADO_MIN)
                        
                    # RAMPA DE ACELERACIÓN Y FRENADO (Corregida)
                    if velocidad_lineal_actual < velocidad_objetivo:
                        # Acelera un ~3% por ciclo para no patinar
                        velocidad_lineal_actual += VELOCIDAD_MAXIMA * 0.03
                        # Limitar para no pasarnos de lo que pide el perfil
                        velocidad_lineal_actual = min(velocidad_lineal_actual, velocidad_objetivo)
                    else:
                        # Si debemos frenar, bajamos instantáneamente a la velocidad objetivo
                        velocidad_lineal_actual = velocidad_objetivo
                        
                    velocidad_base = velocidad_lineal_actual
                    
                    """
                    # Corrección lateral (Repulsión de muros inteligentes)
                    if dist < DIST_APAGADO_LAT:
                        correccion = 0.0
                    else:
                        correccion = 0.0
                        if ps[5] > UMBRAL_PELIGRO_LATERAL:
                            # Repulsión desde la izquierda -> empuja a la derecha
                            correccion = (ps[5] - UMBRAL_PELIGRO_LATERAL) * KP_MUROS_NUEVO
                        elif ps[2] > UMBRAL_PELIGRO_LATERAL:
                            # Repulsión desde la derecha -> empuja a la izquierda
                            correccion = -(ps[2] - UMBRAL_PELIGRO_LATERAL) * KP_MUROS_NUEVO
                    
                    """
                    # Corrección lateral (centrado en pasillo)
                    if dist < DIST_APAGADO_LAT:
                        correccion = 0.0
                    elif ps[5] < UMBRAL_PARED_LIBRE or ps[2] < UMBRAL_PARED_LIBRE:
                        correccion = 0.0
                    else:
                        correccion = (ps[5] - ps[2]) * KP_MUROS
                    #"""

                    left_speed  = velocidad_base - ajuste_rumbo + correccion
                    right_speed = velocidad_base + ajuste_rumbo - correccion

        # META ALCANZADA
        else:
            print("META ALCANZADA!")
            left_speed  = 0.0
            right_speed = 0.0

        # Saturación y accionamiento
        left_speed  = max(min(left_speed,  VELOCIDAD_MAXIMA), -VELOCIDAD_MAXIMA)
        right_speed = max(min(right_speed, VELOCIDAD_MAXIMA), -VELOCIDAD_MAXIMA)
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)

if __name__ == "__main__":
    my_robot = Robot()
    run_robot(my_robot)