%% Paraméterek
% Motor paraméterek
R      = 0.716;      % Armatúra ellenállás [ohm]
L      = 0.00026;    % Armatúra induktivitás [H]
k      = 0.0429;     % Motor állandó (Nm/A vagy V/(rad/s))
J      = 1e-5;       % Tehetetlenségi nyomaték [kg*m^2]
b      = 0.0;        % Viszkózus súrlódás [N*m*s/rad]
T_load = 0.0;        % Terhelő nyomaték [Nm]

% Szabályozó paraméterek
Kp     = 0.5;        % P-szabályozó erősítés
Vmax   = 12.0;       % Feszültség szaturáció (±V)

% Referencia jel (négyszögjel) paraméterek
w_high = 300;        % Felső szögsebesség [rad/s]
w_low  = -300;       % Alsó szögsebesség [rad/s]
freq   = 15;         % Négyszögjel frekvencia [Hz]

% Szimulációs beállítások
t_max         = 0.2;        % Szimulációs idő [s]
dt_controller = 1e-3;       % Szabályozó frissítési időköze [s]

%% Inicializáció
% Kezdeti állapot: x = [i; omega]
x = [0; 0];  
t_current = 0;

% Eredmények tárolása
T_all      = [];
I_all      = [];
Omega_all  = [];
U_all      = [];
Wref_all   = [];

% Négyszögjel periódusa
T_period = 1 / freq;

%% Szimuláció (vezérlő update + adaptív motor integrálás)
while t_current < t_max
    % Következő vezérlő update időpontja
    t_next = min(t_current + dt_controller, t_max);
    
    % Referencia jel számítása a jelenlegi időpillanatban
    if mod(t_current, T_period) < T_period/2
        w_ref = w_high;
    else
        w_ref = w_low;
    end
    
    % Vezérlő: hibaszámítás és feszültség számítás
    error = w_ref - x(2);  % x(2) a motor szögsebessége (omega)
    u_raw = Kp * error;
    u = max(-Vmax, min(u_raw, Vmax)); % Szaturáció
    
    % Motor dinamikája: a vezérlő kimenetét fixnek tekintjük [t_current, t_next]
    motorODE = @(t, x) [ (u - R*x(1) - k*x(2)) / L; ...
                         (k*x(1) - b*x(2) - T_load) / J ];
                     
    opts = odeset('RelTol',1e-6,'AbsTol',1e-9);
    [t_interval, x_interval] = ode45(motorODE, [t_current t_next], x, opts);
    
    % Eredmények tárolása
    T_all      = [T_all; t_interval];
    I_all      = [I_all; x_interval(:,1)];
    Omega_all  = [Omega_all; x_interval(:,2)];
    U_all      = [U_all; u * ones(size(t_interval))];
    Wref_all   = [Wref_all; w_ref * ones(size(t_interval))];
    
    % Kezdőállapot frissítése a következő intervallumhoz
    x = x_interval(end,:)';
    t_current = t_next;
end

%% Ábrázolás
figure;

subplot(4,1,1);
plot(T_all, I_all, 'LineWidth',1.5);
xlabel('Idő [s]');
ylabel('Áram i(t) [A]');
title('DC motor áram');
grid on;

subplot(4,1,2);
plot(T_all, Omega_all, 'LineWidth',1.5);
xlabel('Idő [s]');
ylabel('Szögsebesség ω(t) [rad/s]');
title('DC motor szögsebesség');
grid on;

subplot(4,1,3);
plot(T_all, U_all, 'LineWidth',1.5);
xlabel('Idő [s]');
ylabel('Vezérlő feszültség u(t) [V]');
title('Vezérlő feszültség');
grid on;

subplot(4,1,4);
plot(T_all, Wref_all, 'k--', 'LineWidth',1.5); hold on;
plot(T_all, Omega_all, 'r', 'LineWidth',1.5);
xlabel('Idő [s]');
ylabel('Szögsebesség [rad/s]');
title('Referencia jel és motor szögsebesség');
legend('ω_{ref}(t)', 'ω(t)');
grid on;
