# Proyecto Final ICI4150-1 Robótica y Sistemas Autónomos
## Profesora: Sandra Cano
## Integrantes: Daniel Cornejo, Ian Guerrero, Isidora Osorio
## Línea elegida: Planitifación de rutas
## Objetivo del proyecto
El presente proyecto tiene la finalidad de crear un robot diferencial e-puck con un controlador en Python capaz de seguir una ruta premeditada en una arena del entorno Webots con distintos tipos de obstáculos, el robot debe ser capaz de sobrepasar los obstáculos y seguir la ruta desde inicio a final sin trabarse en los susodichos. Esto se logrará gracias a una robusta implementación de un controlador complejo en el lenguaje de programación de Python que permitirá al robot e-puck sobrepasar los obstáculos gracias a los distintos sensores que posee el robot se puede realizar estimaciones para los movimientos que debe seguir este, como se verá mas adelante en las explicaciones de los distintos sensores y los algoritmos que sigue el robot para tomar distancias y moverse




**Ecuaciones de movimiento:**
### $$v = \frac{v_r + v_l}{2} \tag{1}$$
### $$\omega = \frac{v_r - v_l}{L} \tag{2}$$

**Ecuaciones de estimación de encoders:**
### $$\Delta s_r = r\Delta\theta_r, \quad \Delta s_l = r\Delta\theta_l \tag{3}$$
### $$\Delta s = \frac{\Delta s_r + \Delta s_l}{2}, \quad \Delta\phi = \frac{\Delta s_r - \Delta s_l}{L} \tag{4}$$
### $$x_k = x_{k-1} + \Delta s \cos\left(\phi_{k-1} + \frac{\Delta\phi}{2}\right) \tag{5}$$
### $$y_k = y_{k-1} + \Delta s \sin\left(\phi_{k-1} + \frac{\Delta\phi}{2}\right) \tag{6}$$
### $$\phi_k = \phi_{k-1} + \Delta\phi \tag{7}$$
