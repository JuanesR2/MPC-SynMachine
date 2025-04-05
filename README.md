# 🔋 Control Predictivo (MPC) en Sistema Dinámico de Potencia con CasADi

Este proyecto implementa un controlador predictivo (MPC) utilizando la librería **CasADi** para optimizar el comportamiento dinámico de un sistema eléctrico representado por un modelo simplificado de **máquina síncrona conectada a red**.

---

## ⚙️ Descripción del Modelo

Se considera un sistema con dos variables de estado:

- **ω**: Velocidad angular normalizada  
- **δ**: Ángulo del rotor

Y una variable de control:

- **P_M**: Potencia mecánica inyectada por la máquina

Las ecuaciones del sistema están dadas por:

\[
\begin{cases}
\dot{\omega} = -\frac{P_{max}}{M} \sin(\delta) + \frac{P_M}{M} \\
\dot{\delta} = \omega_b(\omega - 1)
\end{cases}
\]

---

## 📐 Controlador MPC

Se implementa un controlador predictivo con:

- Horizonte de predicción: **N = 10**
- Paso de simulación: **dt = 1 / (60 * 4)**
- Penalización del control: **α = 0.0001**
- Punto de referencia:  
  \[
  x_{ref} = [1, \frac{\pi}{3}], \quad u_{ref} = \frac{10}{15}\sin(\pi/3)
  \]

El controlador minimiza la función de costo cuadrática:

\[
\sum_{k=0}^{N-1} \left\|x_k - x_{ref}\right\|^2 + \alpha \left(u_k - u_{ref}\right)^2
\]

---

## 🧮 Integración y Simulación

Se simula el sistema durante **200 pasos** usando el método de integración **Runge-Kutta de 4to orden**, actualizando el control óptimo en cada paso.

---

## 📊 Resultados

El script genera tres gráficos:

1. **Velocidad angular (ω)**
2. **Ángulo del rotor (δ)**
3. **Potencia mecánica aplicada (P_M)**

Estos resultados muestran cómo el MPC regula el sistema hacia su punto de referencia, asegurando estabilidad y precisión.

---

## 🛠️ Requisitos

- Python 3.x  
- `casadi`, `numpy`, `matplotlib`

```bash
pip install casadi numpy matplotlib
