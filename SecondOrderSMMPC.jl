using JuMP, Ipopt, LinearAlgebra, Plots

# Parámetros del sistema
Wb = 2 * π * 60
Pmax = 10
M = 15

# Definición del modelo de estado T(x, u)
function T(x, u)
    x1, x2 = x[1], x[2]
    u1 = u[1]
    
    dx1 = -Pmax/M * sin(x2) + u1/M
    dx2 = Wb * (x1 - 1)
    
    return [dx1, dx2]
end

# Parámetros de la simulación
nt = 200
nx = 2
nu = 1
dt = 1 / 60 / 4
alpha_param = 0.01

# Punto inicial
xini = [0.8, 0.9]

# Modelo de optimización
model = Model(optimizer_with_attributes(Ipopt.Optimizer))

# Horizonte de predicción
N = 5

# Definir variables de optimización
@variable(model, xk1[1:nx, 1:N])
@variable(model, uopt[1:nu, 1:N])

# Puntos de referencia
xref = [1, π/3]
uref = 10 / 15 * sin(π/3)

# Función objetivo
@objective(model, Min, sum((xk1[:,k] - xref)' * (xk1[:,k] - xref) + alpha_param * (uopt[1,k] - uref)^2 for k in 1:N))

# Restricciones de dinámica del sistema
for k in 2:N
    @constraint(model, xk1[:,k] .== xk1[:,k-1] + dt * T(xk1[:,k-1], uopt[:,k-1]))
end

# Resolver el problema de optimización
optimize!(model)

# Parámetros de simulación
tode = range(0, stop=(nt-1)*dt, length=nt)
xode = zeros(nx, nt)
uode = zeros(nu, nt)

# Punto inicial
x = xini

# Función de control
function ucontrol(x, model, xk1, uopt)
    set_start_value.(xk1[:,1], x)
    optimize!(model)
    u = value(uopt[1,1])
    return u
end

# Guardar el primer paso
xode[:,1] = x
uode[1,1] = ucontrol(x, model, xk1, uopt)

# Simulación utilizando Euler o Runge-Kutta
for k in 2:nt
    global x
    f1 = T(x, [uode[1, k-1]])
    x = x + dt * f1
    u = ucontrol(x, model, xk1, uopt)
    xode[:, k] = x
    uode[1, k] = u
end

# Graficar los resultados de la simulación
p1 = plot(tode, xode[1,:], title="Simulación del sistema: Velocidad angular (ω)", xlabel="t", ylabel="ω")
p2 = plot(tode, xode[2,:], title="Simulación del sistema: Ángulo (δ)", xlabel="t", ylabel="δ")
p3 = plot(tode, uode[1,:], title="Simulación del sistema: Potencia Mecánica (P_M)", xlabel="t", ylabel="P_M")
plot(p1, p2, p3, layout=(3,1))
