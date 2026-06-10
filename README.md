# Proyecto Final ICI4150-1 Robótica y Sistemas Autónomos
## Profesora: Sandra Cano
## Integrantes: Daniel Cornejo, Ian Guerrero, Isidora Osorio
## Línea elegida: ----------------
## Objetivo del proyecto
El presente proyecto tiene la finalidad de crear un robot diferencial e-puck capaz de...




**Ecuaciones de movimiento:**
### $$v = \frac{v_r + v_l}{2} \tag{1}$$
### $$\omega = \frac{v_r - v_l}{L} \tag{2}$$

**Ecuaciones de estimación de encoders:**
### $$\Delta s_r = r\Delta\theta_r, \quad \Delta s_l = r\Delta\theta_l \tag{3}$$
### $$\Delta s = \frac{\Delta s_r + \Delta s_l}{2}, \quad \Delta\phi = \frac{\Delta s_r - \Delta s_l}{L} \tag{4}$$
### $$x_k = x_{k-1} + \Delta s \cos\left(\phi_{k-1} + \frac{\Delta\phi}{2}\right) \tag{5}$$
### $$y_k = y_{k-1} + \Delta s \sin\left(\phi_{k-1} + \frac{\Delta\phi}{2}\right) \tag{6}$$
### $$\phi_k = \phi_{k-1} + \Delta\phi \tag{7}$$
