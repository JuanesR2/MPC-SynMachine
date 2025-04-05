
import casadi.*

% Parametros 
Wb = 2*pi*60;
Pmax = 10;
M = 15;

% Modelo
T = @(x,u) [-Pmax/M*sin(x(2))+u(1)/M; Wb*(x(1)-1)];

% Parametros de la simulación
nt = 1000;
nx = 2;
nu = 1;
dt = 1/60/4;
alpha = 0.0001;

% Punto incial
xini = [0.8;0.9];

%% ------------- Modelo de optimización (Euler) ---------------------------

% Crar el modelo de optimizacion
opti = casadi.Opti();

% Horizonte
N = 10;

% X actual es un parametro debido a que es una medida
xk = opti.parameter(nx,1);
% Las variables de estado 
xk1 = opti.variable(nx,N);
uopt = opti.variable(nu,N);

% Puntos de referencia 
xref = [1;pi/3];
uref = 10/15*sin(pi/3);

% Funcion objetivo del valor 1
L = (xk1(:,1) - xref)'*(xk1(:,1) - xref) + alpha*(uopt(1) - uref)^2;

% Realizamos el OPC
for k = 2:N
    % Sumatoria de la función objetivo 
    L = L + (xk1(:,k) - xref)'*(xk1(:,k) - xref) + alpha*(uopt(k) - uref)^2;
    % Restricciones
    opti.subject_to(xk1(:,k) == xk1(:,k-1) + dt*T(xk1(:,k-1),uopt(:,k-1)));
end 

% Restriccion de la medicion
opti.subject_to(xk1(:,1) == xk);
% Objetivo del problema 
opti.minimize(L);

% Optimizar
opti.solver('ipopt');
% Valor actual de la medición
opti.set_value(xk,[0;0]);
% Solucionar 
sol = opti.solve();


%% ------------- Modelo de simulación (Runge Kutta) -------------------

% Inciializamos vectores
tode = (1:nt)*dt;
xode = zeros(nx,nt);
uode = zeros(nu,nt);

% Punto inicial
x = xini;
u = ucontrol(x,opti, xk, uopt);
% Guardamos el primer paso
xode(:,1) = x;
uode(:,1) = u;

% Arrancamos desde 2
for k=2:nt
    
    % RungeKutta
    f1 = T(x,u);
    f2 = T(x+dt*f1/2,u);
    f3 = T(x+dt*f2/2,u);
    f4 = T(x+dt*f3,u);

    x = x + dt*(f1 + 2*f2 + 2*f3 + f4)/6;
    u = ucontrol(x,opti,xk, uopt);

    xode(:,k) = x;
    uode(:,k) = u;
end

% Graficar los resultados de la simulación
figure;
subplot(3,1,1)
plot(tode, xode(1,:));
ylabel('\omega');
xlabel('t');
title('Simulación del sistema: Velocidad angular (\omega)');
grid on

subplot(3,1,2)
plot(tode, xode(2,:));
ylabel('\delta');
xlabel('t');
title('Simulación del sistema: Ángulo (\delta)');
grid on

subplot(3,1,3)
plot(tode, uode);
ylabel('P_M');
xlabel('t');
title('Simulación del sistema: Potencia Mecánica (P_M)');
grid on



% Funcion de control
function u = ucontrol(x, opti, xk, uopt)
    % Valor medido
    opti.set_value(xk,x);
    % Solucionar 
    sol = opti.solve();
    % Valor de u
    %u = (1/2)*pi/3 %lazo abierto
    u = sol.value(uopt(1));
    %u = 0 ;
end