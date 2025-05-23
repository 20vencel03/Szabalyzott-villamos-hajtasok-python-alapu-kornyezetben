%% PI szabályozott DC motor adaptív integrátorral (ode45) és u feszültség ábrázolása
clear; clc; close all;

%% Paraméterek
% Motor paraméterek
R      = 0.716;      % Armatúra ellenállás [ohm]
L      = 0.00026;    % Armatúra induktivitás [H]
k      = 0.0429;     % Motor állandó (Nm/A vagy V/(rad/s))
J      = 1e-5;       % Tehetetlenségi nyomaték [kg*m^2]
b      = 0.0;        % Viszkózus súrlódás [N*m*s/rad]
T_load = 0.0;        % Terhelő nyomaték [Nm]

% PI szabályozó paraméterek
Kp     = 0.5;        % Arányos erősítés
Ki     = 0.1;        % Integrális erősítés
Vmax   = 12.0;       % Feszültség szaturáció (±V)

% Referencia jel (négyszögjel) paraméterek
w_high = 200;        % Felső szögsebesség [rad/s]
w_low  = -200;       % Alsó szögsebesség [rad/s]
freq   = 15;         % Négyszögjel frekvencia [Hz]

% Szimulációs beállítások
t_max         = 0.2;    % Teljes szimulációs idő [s]
dt_controller = 1e-4;   % Controller frissítési időköz [s]

%% Inicializáció
% Kezdeti állapot: [i; omega]
state = [0; 0];
% Kezdeti integrált hiba
e_int = 0;
% Aktuális idő
t_current = 0;

% Eredmények tárolása
T_all     = [];
I_all     = [];
Omega_all = [];
U_all     = [];
Wref_all  = [];

%% Szimuláció controller intervallumokban
while t_current < t_max
    % Számoljuk ki a referencia négyszögjelet a jelenlegi időpillanatban:
    T_period = 1 / freq;
    if mod(t_current, T_period) < T_period/2
        w_ref = w_high;
    else
        w_ref = w_low;
    end
    
    % Szabályozó számítás:
    % Hibaszámítás: hiba = referencia - aktuális szögsebesség
    error_val = w_ref - state(2);
    % PI szabályozó: u = Kp*error + Ki*integrált hiba
    u = Kp * error_val + Ki * e_int;
    % Szaturáció
    u = max(-Vmax, min(u, Vmax));
    
    % A jelenlegi controller intervallum vége:
    t_interval_end = min(t_current + dt_controller, t_max);
    tspan = [t_current, t_interval_end];
    
    % Motor dinamikájának integrációja ode45-tel (adaptív lépésköz)
    % A bemeneti u értéket az adott intervallumban állandónak feltesszük.
    [T_temp, X_temp] = ode45(@(t, x) motorODE(x, u, R, L, k, J, b, T_load), tspan, state);
    
    % Adatok tárolása
    T_all     = [T_all; T_temp];
    I_all     = [I_all; X_temp(:,1)];
    Omega_all = [Omega_all; X_temp(:,2)];
    U_all     = [U_all; repmat(u, length(T_temp), 1)];
    Wref_all  = [Wref_all; repmat(w_ref, length(T_temp), 1)];
    
    % Frissítjük az állapotot az utolsó integrációs pont értékére
    state = X_temp(end, :)';
    
    % Az integrált hibát a controller intervallum alatt feltesszük, hogy állandó (error_val)
    e_int = e_int + error_val * (t_interval_end - t_current);
    
    % Idő frissítése
    t_current = t_interval_end;
end

%% Ábrázolás
figure;

subplot(4,1,1);
plot(T_all, I_all, 'LineWidth',1.5);
xlabel('Idő [s]');
ylabel('Áram [A]');
title('DC motor áram');
grid on;

subplot(4,1,2);
plot(T_all, Omega_all, 'LineWidth',1.5);
xlabel('Idő [s]');
ylabel('Szögsebesség [rad/s]');
title('DC motor szögsebesség');
grid on;

subplot(4,1,3);
plot(T_all, Wref_all, 'k--', 'LineWidth',1.5); hold on;
plot(T_all, Omega_all, 'r', 'LineWidth',1.5);
xlabel('Idő [s]');
ylabel('Szögsebesség [rad/s]');
title('Referencia jel vs. motor szögsebesség');
legend('ω_{ref}(t)', 'ω(t)');
grid on;

subplot(4,1,4);
plot(T_all, U_all, 'b', 'LineWidth',1.5);
xlabel('Idő [s]');
ylabel('Vezérlő feszültség [V]');
title('U vezérlő feszültség');
grid on;

%% Motor ODE függvény
function dx = motorODE(x, u, R, L, k, J, b, T_load)
    % Állapotvektor: x(1) = i, x(2) = omega
    di_dt = (u - R*x(1) - k*x(2)) / L;
    domega_dt = (k*x(1) - b*x(2) - T_load) / J;
    dx = [di_dt; domega_dt];
end
