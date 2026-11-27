%% SCRIPT DE CONFIGURACIÓN Y SIMULACIÓN - ROVER ACKERMANN (CASCADA 2-DOF)
clear; clc;

% 1. NOMBRE DEL ARCHIVO DE SIMULACIÓN (sin la extensión .slx)
nombre_modelo = 'Simulacion_Rover';

% 2. PARÁMETROS GENERALES DE SIMULACIÓN Y VEHÍCULO
tiempo_sim = 0.5;         % Duración total de la simulación [s]
L_chasis = 0.23965;      % Distancia entre ejes (Wheel base) [m]
D_llanta = 0.109;        % Diámetro de llantas [m]

% 3. SELECCIÓN DE CONTROLADORES ACTIVOS
% 1 = P  |  2 = PD  |  3 = PI  |  4 = PID   |  5 = Relé
modo_control = 3;        % Lazo interno de velocidad (PI 2-DOF)
modo_dir = 5;            % Lazo externo de dirección (PD 2-DOF)

% 4. PARÁMETROS 2-DOF DEL LAZO DE VELOCIDAD (Salida en Voltios [0..3.3V])
% Ganancia total planta: 143.2 RPM/V * (pi*0.109/60) = 0.8173 (m/s)/V
Ts = 0.02;              % Periodo de muestreo lazo velocidad [s] (500 Hz)
u0_vel = 0.8;
h_vel  = 0.5;
Ku = (4*h_vel)/(pi*0.654);
Pu = 0.06;
Kp = Ku/2.1;               % Ganancia Proporcional [V / (m/s)]
Ti = Pu/2.85;            % Tiempo Integral [s] (cancela polo de la planta)
Td = 0.00035;            % Tiempo Derivativo [s] (activo si modo_control = 2 o 4)
N  = 10;                 % Factor del filtro derivativo
b  = 2.7;                  % Ponderación de referencia proporcional [0..1]

% 5. PARÁMETROS 2-DOF DEL LAZO DE DIRECCIÓN (OpenMV H7 / Servo)
Ts_dir = 0.03;           % Periodo de muestreo lazo dirección [s] (50 Hz)
u0_dir = 1.65;           
h_dir  = 0.5;
Ku = (4*h_dir)/(pi*0.654);
Pu = 0.06;
Kp_dir = 2.0;            % Ganancia Proporcional dirección
Ti_dir = 15;            % Tiempo Integral dirección [s] (Kp_dir / Ki_dir)
Td_dir = 0.12;         % Tiempo Derivativo dirección [s] (Kd_dir / Kp_dir)
N_dir  = 4;             % Factor del filtro derivativo dirección
b_dir  = 0.90;            % Ponderación de referencia proporcional dirección


% 6. PARÁMETROS DE TRAYECTORIA Y ACOPLAMIENTO EN CASCADA
% 1 = Círculo  |  2 = Infinito (figura en 8)
tipo_trayectoria = 2;    % Trayectoria de referencia
R_curva = 3;           % Radio de curvatura [m] (menor = curva más cerrada)

V_max = 2;             % Velocidad máxima en recta [m/s]
V_min = 0.05;             % Velocidad mínima en curva cerrada [m/s]
K_freno = 0.82;          % Factor de reducción de velocidad en curva

% 7. CARGAR Y EJECUTAR EL MODELO DE SIMULINK
if ~bdIsLoaded(nombre_modelo)
    load_system(nombre_modelo);
end

nombres_ctrl = {'Controlador P', 'Controlador PD', 'Controlador PI', 'Controlador PID', 'Auto-Tune (Relé)'};
nombres_tray = {'Circular', 'Infinito (figura en 8)'};

fprintf('Iniciando simulación en cascada (Arquitectura 2-DOF)...\n');
fprintf('  -> Velocidad : %s (Kp=%.3f | Ti=%.5f s | Td=%.5f s | b=%.2f | Ts=%.3f s | h=%.2f | u0=%.2f)\n', ...
    nombres_ctrl{modo_control}, Kp, Ti, Td, b, Ts, h_vel, u0_vel);
fprintf('  -> Dirección : %s (Kp_dir=%.2f | Ti_dir=%.2f s | Td_dir=%.4f s | b_dir=%.2f | Ts_dir=%.3f s | h=%.2f | u0=%.2f)\n', ...
    nombres_ctrl{modo_dir}, Kp_dir, Ti_dir, Td_dir, b_dir, Ts_dir, h_dir, u0_dir);
fprintf('  -> Trayectoria: %s | R = %.2f m | V_ref = [%.1f .. %.1f] m/s\n', ...
    nombres_tray{tipo_trayectoria}, R_curva, V_min, V_max);

% Ejecuta la simulación tomando las variables directamente del Workspace
out = sim(nombre_modelo, 'StopTime', num2str(tiempo_sim));

fprintf('Simulación finalizada con éxito.\n\n');

% 8. CÁLCULO DE ÍNDICES DE DESEMPEÑO DEL ERROR (RMSE, IAE, ISE, ITAE)
% --- Lazo de Velocidad ---
t_v = out.err_vel.Time;
e_v = squeeze(out.err_vel.Data);

RMSE_vel = sqrt(mean(e_v.^2));
IAE_vel  = trapz(t_v, abs(e_v));
ISE_vel  = trapz(t_v, e_v.^2);
ITAE_vel = trapz(t_v, t_v .* abs(e_v));

% --- Lazo de Dirección ---
t_d = out.err_dir.Time;
e_d = squeeze(out.err_dir.Data);

RMSE_dir = sqrt(mean(e_d.^2));
IAE_dir  = trapz(t_d, abs(e_d));
ISE_dir  = trapz(t_d, e_d.^2);
ITAE_dir = trapz(t_d, t_d .* abs(e_d));

% Mostrar tabla comparativa en consola
Tabla_Errores = table( ...
    [RMSE_vel; RMSE_dir], ...
    [IAE_vel;  IAE_dir], ...
    [ISE_vel;  ISE_dir], ...
    [ITAE_vel; ITAE_dir], ...
    'VariableNames', {'RMSE', 'IAE', 'ISE', 'ITAE'}, ...
    'RowNames', {['Velocidad (' nombres_ctrl{modo_control} ')'], ...
    ['Dirección (' nombres_ctrl{modo_dir} ')']} );

disp('=== ÍNDICES DE ERROR DE LOS CONTROLADORES ===');
disp(Tabla_Errores);