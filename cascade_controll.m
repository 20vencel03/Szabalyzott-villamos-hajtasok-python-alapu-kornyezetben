%% Paraméterek
% Motor paraméterek
R      = 0.716;      % Armatúra ellenállás [ohm]
L      = 0.00026;    % Armatúra induktivitás [H]
k      = 0.0429;     % Motor állandó (Nm/A vagy V/(rad/s))
J      = 1e-5;       % Tehetetlenségi nyomaték [kg*m^2]
b      = 0.0;        % Viszkózus súrlódás [N*m*s/rad]
T_load = 0.0;        % Terhelő nyomaték [Nm]

% PI szabályozó paraméterek
Kp     = 0.05;        % Arányos erősítés
Ki     = 30;        % Integrális erősítés
Vmax   = 12.0;       % Feszültség szaturáció (±V)

Kp_i = 2;
Ki_i = 300;
Imax=5;

% Referencia jel (négyszögjel) paraméterek
w_high = 200;        % Felső szögsebesség [rad/s]
w_low  = -200;       % Alsó szögsebesség [rad/s]
freq   = 5

;         % Négyszögjel frekvencia [Hz]

% Szimulációs beállítások
t_max         = 0.2;      % Teljes szimulációs idő [s]
dt_controller = 1e-4;     % Szabályozó frissítési időköz [s]

%% Inicializáció
% Kezdeti állapot: [i; omega]
state = [0; 0];
% Kezdeti integrált hiba
e_int = 0;
e_int_w =0;
% Aktuális idő
t_current = 0;

% Eredmények tárolása
T_all     = [];
I_all     = [];
Omega_all = [];
U_all     = [];
Wref_all  = [];
error_all = [];

%% Szimuláció (controller update + motor integrálás)
while t_current < t_max
    % Négyszögjel periódus
    T_period = 1 / freq;
    % Referenciajel (négyszög) számítása
    if mod(t_current, T_period) < T_period/2
        w_ref = w_high;
    else
        w_ref = w_low;
    end
    
    % Hibaszámítás: hiba = referencia - aktuális szögsebesség
    error_val_w = w_ref - state(2);
    
    % --- PI rész ---
    % A tiszta PI-kimenet (diszkrét frissítés közelítésével):
    u_pi_w = Kp * ( error_val_w + Ki*dt_controller*(e_int_w + error_val_w) );

    % --- Indukált feszültség kompenzálása (feed-forward) ---
    % A motor 'k' állandójával és a mért szögsebességgel kiegészítjük a jelet:
    %u_ff = k * state(2);  % a back-EMF közelítő kompenzálása
    
    % Teljes vezérlője

    % Szaturáció (±Vmax)
    if u_pi_w > Imax
        u_pi_w = Imax;
        int_flag = false;  % integrátor fagyasztása
    elseif u_pi_w < -Imax
        u_pi_w = -Imax;
        int_flag = false;
    else
        int_flag = true;
    end
    
    % Integrátor frissítése (ha nincs telítés)
    if int_flag
        e_int_w = e_int_w + error_val_w;
    end

    I_ref = u_pi_w;

     % Hibaszámítás: hiba = referencia - aktuális szögsebesség
    error_val = I_ref - state(1);
    
    % --- PI rész ---
    % A tiszta PI-kimenet (diszkrét frissítés közelítésével):
    u_pi = Kp_i * ( error_val + Ki_i*dt_controller*(e_int + error_val) );

    % --- Indukált feszültség kompenzálása (feed-forward) ---
    % A motor 'k' állandójával és a mért szögsebességgel kiegészítjük a jelet:
    u_ff = k * state(2);  % a back-EMF közelítő kompenzálása
    
    % Teljes vezérlőjel:
    u_raw = u_pi;

    % Szaturáció (±Vmax)
    if u_raw > Vmax
        u = Vmax;
        int_flag = false;  % integrátor fagyasztása
    elseif u_raw < -Vmax
        u = -Vmax;
        int_flag = false;
    else
        u = u_raw;
        int_flag = true;
    end
    
    % Integrátor frissítése (ha nincs telítés)
    if int_flag
        e_int = e_int + error_val;
    end
    
    % A következő intervallum vége
    t_interval_end = min(t_current + dt_controller, t_max);
    
    % Motor dinamikájának integrációja ode45-tel (állandónak vett u)
    [T_temp, X_temp] = ode45(@(t, x) motorODE(x, u, R, L, k, J, b, T_load), ...
                             [t_current, t_interval_end], state);
    
    % Adatok tárolása
    T_all     = [T_all; T_temp];
    I_all     = [I_all; X_temp(:,1)];
    Omega_all = [Omega_all; X_temp(:,2)];
    U_all     = [U_all; repmat(u, length(T_temp), 1)];
    Wref_all  = [Wref_all; repmat(w_ref, length(T_temp), 1)];
    error_all = [error_all; repmat(error_val, length(T_temp), 1)];
    
    % Utolsó állapot beállítása a következő ciklushoz
    state = X_temp(end, :)';
    t_current = t_interval_end;
end

%% Ábrázolás
figure('Name','Indukált feszültség kompenzált PI-szabályozás');
sgtitle('DC motor PI szabályzó + visszaható kompenzálás');

subplot(4,1,1);
plot(T_all, I_all, 'LineWidth',1.5);
xlabel('Idő [s]');
ylabel('Áram [A]');
title('DC motor áram');
grid on;

subplot(4,1,2);
plot(T_all, error_all, 'LineWidth',1.5);
xlabel('Idő [s]');
ylabel('Hiba [rad/s]');
title('Szögsebesség hiba');
grid on;

subplot(4,1,3);
plot(T_all, Wref_all, 'k--', 'LineWidth',1.5); hold on;
plot(T_all, Omega_all, 'r', 'LineWidth',1.5);
xlabel('Idő [s]');
ylabel('Szögsebesség [rad/s]');
title('Referencia és motor szögsebesség');
legend('ω_{ref}', 'ω');
grid on;

subplot(4,1,4);
plot(T_all, U_all, 'b', 'LineWidth',1.5);
xlabel('Idő [s]');
ylabel('Vezérlő feszültség [V]');
title('Vezérlőjel (PI + k·ω kompenzáció)');
grid on;

%% Motor ODE függvény
function dx = motorODE(x, u, R, L, k, J, b, T_load)
    % x(1) = i, x(2) = omega
    di_dt = (u - R*x(1) - k*x(2)) / L;
    domega_dt = (k*x(1) - b*x(2) - T_load) / J;
    dx = [di_dt; domega_dt];
end
