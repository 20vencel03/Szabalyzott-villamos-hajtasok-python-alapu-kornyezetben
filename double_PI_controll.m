%% Kétszintű szabályozású DC motor példa (külső + belső)
clear; clc; close all;

%% 1) Paraméterek
% Motor paraméterek
R      = 0.716;      % Armatúra ellenállás [ohm]
L      = 0.00026;    % Armatúra induktivitás [H]
k      = 0.0429;     % Motor állandó (Nm/A vagy V/(rad/s))
J      = 1e-5;       % Tehetetlenségi nyomaték [kg*m^2]
b      = 0.0;        % Viszkózus súrlódás [N*m*s/rad]
T_load = 0.0;        % Terhelő nyomaték [Nm]

% "Belső" PI szabályozó paraméterek 
Kp      = 0.7;       % Arányos erősítés
Ki      = 1000;       % Integrális erősítés
Vmax    = 12.0;      % Abszolút lehetséges max feszültség

% "Belső" PI szabályozó paraméterek 
Kp_outer      = 0.05;       % Arányos erősítés
Ki_outer      = 50;       % Integrális erősítés
Imax    = 3.0;      % Abszolút lehetséges max áram

% A külső szabályozó - a példa kedvéért - két feszültségkorlátot váltogat
Vmax_lower = -Vmax/3; % Alsó feszültségkorlát (külső szabályozó adja)
Vmax_upper = Vmax/3;   % Felső feszültségkorlát

% Négyszögjel alakú sebesség-referencia
w_high   = 200;      % Felső szögsebesség [rad/s]
w_low    = -200;     % Alsó szögsebesség [rad/s]
freq     = 5;       % Négyszögjel frekvencia [Hz]

% Időzítések
t_max         = 0.2;    % Szimulációs idő [s]
dt_controller = 1e-4;   % Belső szabályozó időlépése
dt_outer      = 1e-3;   % Külső szabályozó (pl. 100-szor ritkább)

%% 2) Inicializáció
% Kezdeti állapot: [áram; sebesség]
state     = [0; 0];
e_int     = 0;          % PI integrált hiba
t_current = 0;
u = 0;
error_val = 0;
error_val_outer = 0;
I_ref = 0;
e_int_outer=0;

% A külső szabályozó következő aktiválási időpontja
t_outer_next = dt_outer;
t_inner_next = dt_controller;

% Kezdetben a külső szabályozó feszültséglimitje legyen a nagyobbik
Vmax_current = Vmax_upper;
% Így indulunk: 12 V telítés a belső szabályozóban

% Eredmények tárolása
T_all     = [];
I_all     = [];
I_ref_all = [];
Omega_all = [];
U_all     = [];
Wref_all  = [];
Error_all = [];
Error_all_outer = [];
Eint_all  = [];
Vmax_all  = [];  % A tényleges, külső szabályozó által adott fesz. korlát
Imax_all = [];    % A tényleges, külső szabályozó által adott áram. korlát
I_all_outer = [];



%% 3) Szimuláció
while t_current < t_max
        % Négyszögjel periódus számítása
    T_period = 1/freq;
    if mod(t_current, T_period) < T_period/2
        w_ref = w_high;
    else
        w_ref = w_low;
    end


    % 3.1) Külső szabályozó frissítése, ha eljött az ideje (t_outer_next)
    if t_current >= t_outer_next
        %
        % if abs(I_all_outer - I_all) < 1e-9
        %     Vmax_current = Vmax_lower; 
        % else
        %     Vmax_current = Vmax_upper; 
        % end
        % Újraütemezzük a következő külső szabályozói frissítést:
        t_outer_next = t_outer_next + dt_outer;
    
    
            % 3.2) Belső szabályozó  - sebesség alapú PI
        
            
            % Hiba
            error_val_outer = w_ref - state(2);
            
            % Diszkrét PI kimenet
            I_raw = Kp_outer * ( error_val_outer + Ki_outer * dt_outer * ( e_int_outer + error_val_outer ) );
            
            % Telítés (kisebb a mindenkori abszolút Vmax értéknél, de a külső hurok
            % ezt még "szűkítheti" Vmax_current-re)
            if I_raw > Imax
                I_ref = Imax;
                int_flag = false;  
            elseif I_raw < -Imax
                I_ref = -Imax;
                int_flag = false; 
            else
                I_ref = I_raw;
                int_flag = true; 
            end
            
            % Integrátor frissítése (ha nem telített)
            if int_flag
                e_int_outer = e_int_outer + error_val_outer;
            end
    end
    
    
    %3.3.a Belső kör

    if t_current >= t_inner_next
        %
        % if abs(I_all_outer - I_all) < 1e-9
        %     Vmax_current = Vmax_lower; 
        % else
        %     Vmax_current = Vmax_upper; 
        % end
        % Újraütemezzük a következő belső szabályozói frissítést:
        t_inner_next = t_inner_next + dt_controller;
    
    
            % 3.2) Belső szabályozó  - sebesség alapú PI
        
            
            % Hiba
            error_val = I_ref - state(1);
            
            % Diszkrét PI kimenet
            u_raw = Kp * ( error_val + Ki* dt_controller * ( e_int + error_val ) );
            
            % Telítés (kisebb a mindenkori abszolút Vmax értéknél, de a külső hurok
            % ezt még "szűkítheti" Vmax_current-re)
            if u_raw > Vmax_upper
                u_PI = Vmax_upper;
                int_flag = false;  
            elseif u_raw < Vmax_lower
                u_PI = Vmax_lower;
                int_flag = false; 
            else
                u_PI = u_raw;
                int_flag = true; 
            end
            
            % Integrátor frissítése (ha nem telített)
            if int_flag
                e_int = e_int + error_val;
            end

            u = u_PI + k*state(2);
    end


    
    % 3.3) Motor dinamikájának integrálása (ode45, dt_controller intervallum)
    t_interval_end = min(t_current + dt_controller, t_max);
    tspan = [t_current, t_interval_end];
    
    [T_temp, X_temp] = ode45(@(t, x) motorODE(x, u, R, L, k, J, b, T_load), ...
                             tspan, state);
    
    % 3.4) Adatok elmentése
    T_all       = [T_all; T_temp];
    I_all       = [I_all; X_temp(:,1)];
    Omega_all   = [Omega_all; X_temp(:,2)];
    U_all       = [U_all; repmat(u, length(T_temp),1)];
    Wref_all    = [Wref_all; repmat(w_ref, length(T_temp),1)];
    Error_all   = [Error_all; repmat(error_val, length(T_temp),1)];
    Error_all_outer   = [Error_all_outer; repmat(error_val_outer, length(T_temp),1)];
    Eint_all    = [Eint_all; repmat(e_int, length(T_temp),1)];
    Vmax_all    = [Vmax_all; repmat(Vmax, length(T_temp),1)];
    I_ref_all   = [I_ref_all; repmat(I_ref, length(T_temp),1)];
    
    
    % Állapot frissítése a következő ciklushoz
    state     = X_temp(end, :)';
    t_current = t_interval_end;
end

%% 4) Eredmények ábrázolása
figure('Name','Kétszintű DC motor szabályozás');
sgtitle('DC motor – gyors PI + lassú váltogatott V_{max}');

subplot(5,1,1);
plot(T_all, I_all, 'LineWidth',1.2);
hold on 
plot(T_all, I_ref_all);
xlabel('Idő [s]');
ylabel('i [A]');
title('Motor áram');
grid on;

subplot(5,1,2);
plot(T_all, Error_all, 'LineWidth',1.2);
xlabel('Idő [s]');
ylabel('Hiba [A]');
title('aramhiba');
grid on;

subplot(5,1,3);
plot(T_all, Wref_all, 'k--','LineWidth',1.2); hold on;
plot(T_all, Omega_all, 'r','LineWidth',1.2);
xlabel('Idő [s]');
ylabel('\omega [rad/s]');
title('Referencia vs. motor szögsebesség');
legend('\omega_{ref}','\omega');
grid on;

subplot(5,1,4);
plot(T_all,U_all,'b','LineWidth',1.2);
hold on;
plot(T_all, Vmax_all,'m--','LineWidth',1.0);
plot(T_all, -Vmax_all,'m--','LineWidth',1.0);
xlabel('Idő [s]');
ylabel('Feszültség [V]');
title('Belső PI kimenet és aktuális telítés (külső hurok)');
legend('u(t)','+V_{max,current}','-V_{max,current}');
grid on;

subplot(5,1,5);
plot(T_all, Error_all_outer,'LineWidth',1.2);
xlabel('Idő [s]');
ylabel('e_{int}');
title('PI integrált sebesség hiba');
grid on;

%% 5) ODE függvény
function dx = motorODE(x, u, R, L, k, J, b, T_load)
    % x(1) = i, x(2) = omega
    i     = x(1);
    omega = x(2);
    di_dt     = (u - R*i - k*omega) / L;
    domega_dt = (k*i - b*omega - T_load) / J;
    dx = [di_dt; domega_dt];
end
