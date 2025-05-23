%% Paraméterek
% Motor paraméterek (PMSM)
R      = 0.716;      % Áramkör ellenállás [ohm]
L      = 0.00026;    % Induktivitás (L_d = L_q = L) [H]
psi_f  = 0.0429;     % Állandó fluxus [Wb] (egyszerűsítés céljából ugyanaz, mint a DC motor esetében)
p      = 2;          % Polepárok száma
J      = 4e-5;       % Tehetetlenségi nyomaték [kg*m^2]
b      = 0.0;        % Viszkózus súrlódás [N*m*s/rad]
T_load = 0.0;        % Terhelő nyomaték [Nm]

% PI szabályozó paraméterek
% Külső szabályozó (szögsebesség szabályozás)
Kp_speed = 0.1;
Ki_speed = 50;

% Belső szabályozók (áram szabályozás)
% d-tengely (i_d referencia: 0)
Kp_id = 0.5;
Ki_id = 1000;
% q-tengely (i_q referencia a külső szabályozó kimenete)
Kp_iq = 0.5;
Ki_iq = 1000;

Vmax   = 24.0;       % Feszültség szaturáció (±V)

% Referencia jel (négyszögjel a szögsebességre)
w_high = 200;       % [rad/s]
w_low  = -200;      % [rad/s]
freq   = 15;        % Frekvencia [Hz]

% Szimulációs beállítások
t_max         = 0.2;      % Szimulációs idő [s]
dt_controller = 1e-4;     % Szabályozó frissítési időköz [s]

%% Inicializáció
% Állapotvektor: [i_d; i_q; omega; theta]
state = [0; 0; 0; 0];

% Integrált hibák (külső és belső szabályozókhoz)
e_int_speed = 0;   % Szögsebesség hibaintegrál
e_int_id    = 0;   % d-tengely áram hiba integrál
e_int_iq    = 0;   % q-tengely áram hiba integrál

Poles_n = 2;

t_current = 0;

% Eredmények tárolása
T_all         = [];
Id_all        = [];
Iq_all        = [];
Omega_all     = [];
Theta_all     = [];
Vd_all        = [];
Vq_all        = [];
Wref_all      = [];
Speed_error_all = [];
Iq_ref_all    = [];
I_x = [];
I_y = [];
I_a = [];
I_b = [];
I_c = [];


%% Szimuláció (külső és belső szabályozás, majd a motor integrációja)
while t_current < t_max
    % Külső (szögsebesség) referencia: négyszögjel
    T_period = 1 / freq;
    if mod(t_current, T_period) < T_period/2
        w_ref = w_high;
    else
        w_ref = w_low;
    end
    
    % --- Külső szabályozás (szögsebesség PI) ---
    speed_error = w_ref - state(3);
    % Diszkrét PI kimenet (egyszerűsített frissítéssel)
    u_speed = Kp_speed * ( speed_error + Ki_speed*dt_controller*(e_int_speed + speed_error) );
    e_int_speed = e_int_speed + speed_error;
    
    % A speed PI kimenet nyomatékreferenciaként működik, amelyből a q-tengely áram referencia:
    T_ref = u_speed;
    i_q_ref = T_ref / ((3/2)*p*psi_f);
    
    % d-tengely referencia (feltételezzük, hogy 0 a kívánt érték)
    i_d_ref = 0;
    
    % --- Belső szabályozás (áram PI-k) ---
    % d-tengely
    error_id = i_d_ref - state(1);
    u_id = Kp_id * ( error_id + Ki_id*dt_controller*(e_int_id + error_id) );
    % q-tengely
    error_iq = i_q_ref - state(2);
    u_iq = Kp_iq * ( error_iq + Ki_iq*dt_controller*(e_int_iq + error_iq) );
    
    % Feed-forward kompenzáció a kereszthatás ellensúlyozására
    v_d_ff = Poles_n*L * state(2) * state(3);            % d-tengelyen
    v_q_ff = Poles_n*(L * state(1) * state(3) + psi_f * state(3));  % q-tengelyen
    
    % Teljes feszültségparancsok
    v_d_raw = u_id - v_d_ff;
    v_q_raw = u_iq + v_q_ff;
    
    % Szaturáció vizsgálata (egyenként)
    if v_d_raw > Vmax
        v_d = Vmax;
        flag_id = false;
    elseif v_d_raw < -Vmax
        v_d = -Vmax;
        flag_id = false;
    else
        v_d = v_d_raw;
        flag_id = true;
    end
    
    if v_q_raw > Vmax
        v_q = Vmax;
        flag_iq = false;
    elseif v_q_raw < -Vmax
        v_q = -Vmax;
        flag_iq = false;
    else
        v_q = v_q_raw;
        flag_iq = true;
    end
    
    % Belső integrátorok frissítése, ha nincs telítés
    if flag_id
        e_int_id = e_int_id + error_id;
    end
    if flag_iq
        e_int_iq = e_int_iq + error_iq;
    end
    
    % Időintegrálás (dt_controller) az aktuális feszültségparancsokkal
    t_interval_end = min(t_current + dt_controller, t_max);
    [T_temp, X_temp] = ode45(@(t, x) pmsmODE(x, v_d, v_q, R, L, psi_f, p, J, b, T_load), ...
                             [t_current, t_interval_end], state);
    %RRF to SRF
    theta=state(4)*Poles_n;
    i_vector = [cos(theta) -sin(theta);
        sin(theta) cos(theta)]*[state(1); state(2)];
    i_a=i_vector(1);
    i_b = (sqrt(3)*i_vector(2)-i_vector(1))/2;
    i_c = -i_a-i_b;

    % Eredmények tárolása
    T_all         = [T_all; T_temp];
    Id_all        = [Id_all; X_temp(:,1)];
    Iq_all        = [Iq_all; X_temp(:,2)];
    Omega_all     = [Omega_all; X_temp(:,3)];
    Theta_all     = [Theta_all; X_temp(:,4)];
    Vd_all        = [Vd_all; repmat(v_d, length(T_temp), 1)];
    Vq_all        = [Vq_all; repmat(v_q, length(T_temp), 1)];
    Wref_all      = [Wref_all; repmat(w_ref, length(T_temp), 1)];
    Speed_error_all = [Speed_error_all; repmat(speed_error, length(T_temp), 1)];
    Iq_ref_all    = [Iq_ref_all; repmat(i_q_ref, length(T_temp), 1)];
    I_x =[I_x; repmat(i_vector(1), length(T_temp), 1)];
    I_y =[I_y; repmat(i_vector(2), length(T_temp), 1)];
    I_a =[I_a; repmat(i_a, length(T_temp), 1)];
    I_b =[I_b; repmat(i_b, length(T_temp), 1)];
    I_c =[I_c; repmat(i_c, length(T_temp), 1)];

    
    % Állapot és idő frissítése a következő ciklushoz
    state = X_temp(end, :)';
    t_current = t_interval_end;
end

%% Ábrázolás
figure('Name','PMSM motor: Két tengelyes PI szabályozás');
sgtitle('PMSM motor PI szabályozás: belső (áram) és külső (szögsebesség)');

subplot(6,1,1);
plot(T_all, Id_all, 'LineWidth',1.5);
xlabel('Idő [s]');
ylabel('i_d [A]');
title('d-tengely áram');
grid on;

subplot(6,1,2);
plot(T_all, Iq_all, 'LineWidth',1.5);
xlabel('Idő [s]');
ylabel('i_q [A]');
title('q-tengely áram');
grid on;

subplot(6,1,3);
plot(T_all, Omega_all, 'LineWidth',1.5);
xlabel('Idő [s]');
ylabel('Szögsebesség [rad/s]');
title('Motor szögsebesség');
grid on;

subplot(6,1,4);
plot(T_all, Vd_all, 'LineWidth',1.5);
xlabel('Idő [s]');
ylabel('v_d [V]');
title('d-tengely feszültség');
grid on;

subplot(6,1,5);
plot(T_all, Vq_all, 'LineWidth',1.5);
xlabel('Idő [s]');
ylabel('v_q [V]');
title('q-tengely feszültség');
grid on;
hold off;


subplot(6,1,6);
plot(T_all, I_a, 'LineWidth',1.5);
hold on;
plot(T_all, I_b, 'LineWidth',1.5);

plot(T_all, I_c, 'LineWidth',1.5);
xlabel('Idő [s]');
ylabel('i_a, i_b. i_c [A]');
title('Fázisáramok');
grid on;

%% PMSM ODE függvény
function dx = pmsmODE(x, v_d, v_q, R, L, psi_f, p, J, b, T_load)
    % Állapot: x(1)=i_d, x(2)=i_q, x(3)=omega, x(4)=theta
    i_d   = x(1);
    i_q   = x(2);
    omega = x(3);
    
    % d- és q tengely egyenletek (L_d = L_q = L feltételezve)
    di_d_dt = (v_d - R*i_d + L*i_q*omega) / L;
    di_q_dt = (v_q - R*i_q - L*i_d*omega - psi_f*omega) / L;
    
    % Elektromágneses nyomaték (egyszerűsítve: T_e = (3/2)*p*psi_f*i_q)
    T_e = (3/2)*p*psi_f*i_q;
    
    % Mechanikai egyenletek
    domega_dt = (T_e - T_load - b*omega) / J;
    dtheta_dt = omega;
    
    dx = [di_d_dt; di_q_dt; domega_dt; dtheta_dt];
end
