# Proyecto Final ICI4150-1 Robótica y Sistemas Autónomos
## Profesora: Sandra Cano
## Integrantes: Daniel Cornejo, Ian Guerrero, Isidora Osorio
## Línea elegida: Planificación de rutas
## Objetivo del proyecto
El presente proyecto tiene la finalidad de crear un robot diferencial e-puck con un controlador en Python capaz de seguir una ruta premeditada en una arena del entorno Webots con distintos tipos de obstáculos, el robot debe ser capaz de sobrepasar los obstáculos y seguir la ruta desde inicio a final sin trabarse en los susodichos. Esto se logrará gracias a una robusta implementación de un controlador complejo en el lenguaje de programación de Python que permitirá al robot e-puck sobrepasar los obstáculos gracias a los distintos sensores que posee el robot se puede realizar estimaciones para los movimientos que debe seguir este, como se verá mas adelante en las explicaciones de los distintos sensores y los algoritmos que sigue el robot para tomar distancias y moverse en la arena.

## Descripción del robot, sensores y actuadores utilizados
A continuación se presentan distintos datos del robot e-puck que son relevantes. Para el desarrollo de este proyecto se utilizó el simulador Webots modelando un robot e-puck, el cual cuenta con una configuración de tracción diferencial. A continuación, se detallan sus características físicas, así como los sensores y actuadores que se habilitaron en el controlador para resolver la navegación en el laberinto.

### 1. Características Físicas y Cinemáticas
El modelo cinemático del robot está configurado con las siguientes dimensiones físicas fundamentales para el cálculo de la odometría
- **Radio de las ruedas**: 0.0205m.
- **Distancia entre las ruedas**: 0.052m.
- **Velocidad máxima de los motores**: 6.28rad/s.

### 2. Actuadores
El movimiento del robot se logra mediante dos motores de corriente continua (DC) independientes, uno para cada rueda. Los motores de las ruedas izquierda y derecha actúan bajo control de velocidad (setVelocity). Durante la navegación normal, la velocidad base se ajusta a un 40% de la capacidad máxima (aprox. 2.51 rad/s) para el avance, y un 20% (aprox. 1.25 rad/s) durante las rutinas de rotación para asegurar precisión. En situaciones de evasión, los motores invierten su giro al 50% de su capacidad.

### 3. Sensores
Para lograr una navegación autónoma y conocer el estado interno y externo del robot, el controlador hace uso de dos tipos de sensores principales

- **Sensores de posición (encoders)**: Se habilitaron los sensores angulares de ambas ruedas (left wheel sensor y right wheel sensor). Son esenciales para la odometría. El controlador calcula la variación del ángulo de giro en cada iteración del simulador para estimar el desplazamiento lineal ($\Delta s$) y la rotación del chasis ($\Delta \phi$). Esto permite mantener un seguimiento continuo de la posición global del robot ($x$, $y$, $\theta$) respecto a su punto de inicio.
- **Sensores de proximidad (infrarrojos)**: El e-puck cuenta con un anillo de 8 sensores infrarrojos (ps0 a ps7), los cuales se habilitaron en su totalidad, aunque el algoritmo de control focaliza su toma de decisiones en cuatro de ellos:
  - Sensores frontales (ps7 y ps0): Se utilizan para detectar peligro de colisión inminente (obstáculos de frente). Si el valor de lectura supera el umbral de 400.0, el robot pasa a un estado de evasión rápida.
  - Sensores laterales (ps6 izquierdo y ps1 derecho): Sirven para la corrección de trayectoria. Si la lectura supera el umbral de 150.0 (indicando demasiada cercanía a una pared del laberinto), el controlador inyecta una     corrección en las velocidades de las ruedas para mantener al robot centrado en el pasillo.

### 4. Lógica de control
El controlador implementa una arquitectura híbrida:

- **Planificación Global**: Al iniciar, transforma la matriz del laberinto utilizando el algoritmo A* para calcular la ruta óptima desde el punto de inicio hasta la meta. Los nodos lógicos se mapean a coordenadas físicas exactas (waypoints).
- **Control Local (Máquina de Estados)**: El robot alterna entre tres estados de navegación:
  - Rotando: Gira sobre su propio eje hasta alinearse con el ángulo del siguiente waypoint (con una tolerancia de 0.08 radianes).
  - Avanzando: Se desplaza hacia el waypoint utilizando un control Proporcional (Kp = 2.0) sobre el error angular para ajustar el rumbo dinámicamente, al mismo tiempo que suma o resta velocidades si detecta paredes laterales.
  - Evadiendo: Rutina de emergencia que interrumpe la navegación normal para evitar choques inminentes mediante giros bruscos.

## Descripción de los escenarios de prueba 
En esta sección se presentan detalladamente los escenarios de prueba en el entorno de Webots . . .

## Explicación del algoritmo utilizado
Para seguir con la explicación, se hablará del algoritmo que contiene el controlador del robot e-puck realizado en el lenguaje Python . . .

## Pseudocódigo de la solución
A continuación se muestra el pseudocódigo del algoritmo que se implementó . . .

## Resultados obtenidos

Demostración de laberinto dificultad media:

<p align="center">
  <a href="https://youtu.be/11VBAuZGZaM">
    <img src="./imagenes/miniatura.png" alt="Ver video en YouTube" width="80%">
  </a>
</p>

Se presentarán los resultados obtenidos con el robot y la ruta seguida. . .

## Gráficos
. . .

## ¿Cómo ejecutar la simulación?
Se debe seguir este paso a paso para poder ejecutar la simulación del robot en el entorno Webots

## Conclusiones finales
Para terminar, se hablará de lo que se puede concluir con el proyecto. . .


**Ecuaciones de movimiento:**
### $$v = \frac{v_r + v_l}{2} \tag{1}$$
### $$\omega = \frac{v_r - v_l}{L} \tag{2}$$

**Ecuaciones de estimación de encoders:**
### $$\Delta s_r = r\Delta\theta_r, \quad \Delta s_l = r\Delta\theta_l \tag{3}$$
### $$\Delta s = \frac{\Delta s_r + \Delta s_l}{2}, \quad \Delta\phi = \frac{\Delta s_r - \Delta s_l}{L} \tag{4}$$
### $$x_k = x_{k-1} + \Delta s \cos\left(\phi_{k-1} + \frac{\Delta\phi}{2}\right) \tag{5}$$
### $$y_k = y_{k-1} + \Delta s \sin\left(\phi_{k-1} + \frac{\Delta\phi}{2}\right) \tag{6}$$
### $$\phi_k = \phi_{k-1} + \Delta\phi \tag{7}$$
