% DC motor szimuláció

% --- Motorparaméterek ---
R = 0.716;        % Armatúra ellenállás [Ohm]
L = 0.00026;      % Armatúra induktivitás [H]
k = 0.0429;       % Motor állandó [Nm/A vagy V/(rad/s)]
J = 1e-5;         % Tehetetlenségi nyomaték [kg·m^2]
b = 0;            % Viszkózus súrlódás [N·m·s/rad], jelen esetben 0
% Ha van terhelő nyomaték (T_load), azt is belefoglalhatod a képletekbe,
% de itt 0-nak vesszük.

% --- Állapottér leírás ---
% Állapotvektor: x = [i; omega], ahol
%   i     = armatúraáram [A]
%   omega = motor szögsebessége [rad/s]
%
% Egyenletek:
%   L * di/dt      = u - R*i - k*omega
%   J * d(omega)/dt= k*i - b*omega   (T_load=0 esetén)
%
% Ez átrendezve:
%   di/dt = (1/L)*u  - (R/L)*i  - (k/L)*omega
%   d(omega)/dt = (k/J)*i - (b/J)*omega
%
% -> A mátrixok:
A = [ -R/L   -k/L
       k/J    -b/J ];

B = [ 1/L
      0   ];

% Kimenetként (C mátrix) most mindkét állapotot szeretnénk figyelni:
%  y1 = i(t)
%  y2 = omega(t)
C = eye(2);  
D = [0;0];

% --- Szimuláció beállításai ---
% 12 V-os lépés-bemenet, 0.2 s időtartam, sűrű időfelbontás
t_max = 0.2;
dt    = 1e-5;          % Időlépés
t     = 0 : dt : t_max;

u     = 12*ones(size(t));  % 12 V állandó feszültség
x0    = [0; 0];            % Kezdeti feltételek (i(0)=0, omega(0)=0)

% --- Rendszer létrehozása és futtatása ---
sys = ss(A,B,C,D);      % állapottér-modell a MATLAB-ban
[y, t_out, x_out] = lsim(sys, u, t, x0);

% A kimenet y(:,1) az áram, y(:,2) a szögsebesség

% --- Eredmények ábrázolása ---
figure;

subplot(2,1,1);
plot(t_out, y(:,1), 'LineWidth',1.2);
grid on;
xlabel('Idő [s]');
ylabel('Áram [A]');
title('Armatúra áram (i)');

subplot(2,1,2);
plot(t_out, y(:,2), 'r', 'LineWidth',1.2);
grid on;
xlabel('Idő [s]');
ylabel('Szögsebesség [rad/s]');
title('Motor szögsebesség (ω)');

sgtitle('DC motor szimuláció a megadott paraméterekkel');
