%%
% Author: 	Eduardo Cáceres de la Calle
% Subject: 	Control Systems Design
% Degree: 	Industrial Electronics and Automatic Control Engineering
% University:	Universidad de Valladolid (UVa) - EII
%
% Code written in 2016
% Uploaded for educational purposes only, don't be too hard on me :)


%% Laboratorio Tema 6

close all
clear
clc

% Funcion de transferencia con la que trabajaremos:
num=[10];
den=[0.01, 0.25, 1, 0];
disp('Función de transferencia en lazo abierto:')
G_la=tf(num,den)

% Definiciones e inicializaciones
i=1;
t=-10:0.001:10;
ramp=0:0.001:500;
Y=heaviside(t);     % 0 si t<0, 1/2 si t=0, 1 si t>0 -> señal de control

num_R=[1];                % Construcción de la f.t. de rampa
den_R=[1, 0];
R=tf(num_R,den_R);



% **Ejercicio 1**

% Estudio previo de la f.t:

%   Lazo abierto

figure(i)
subplot(2,1,1)
plot(t,Y)
hold on
step(G_la, 'r')         % Respuesta de G ante un escalón
axis([-2 10 -2 30])
title('Respuesta ante escalon en lazo abierto')
legend('Entrada escalón', 'Respuesta del sistema', 'Location', 'northeast')
hold off

pause

%   Lazo cerrado con realimentación unidad
[num_lc, den_lc]= cloop(num, den);
disp('Función de transferencia en lazo cerrado:')
G_lc=tf(num_lc, den_lc)

subplot(2,1,2)
plot(t,Y)
hold on
step(G_lc, 'r')
axis([-2 8 -0.5 2])
title('Respuesta ante escalon en lazo cerrado')
legend('Entrada escalón', 'Respuesta del sistema', 'Location', 'southeast')
hold off
pause


%   Error ante entrada rampa
i=i+1;
figure(i)
plot(ramp,ramp);        % Señal de control: rampa
hold on;

Rampa=series(R,G_lc)    % Respuesta ante rampa
step(Rampa,'r')
axis([0 5 0 5])
title('Respuesta ante rampa (en lazo cerrado)')
legend('Entrada rampa', 'Respuesta del sistema', 'Location', 'northwest')
hold off;
pause


% Diseño del controlador:

clc
disp('Función de transferencia en lazo abierto:')
G_la

i=i+1;
figure(i)
margin(G_la)

pause                           % Captura con puntos
                   
%   Calculo de Kp para MF>55
disp('Calculos para MF=60')
disp('(12.5-0)= 20log(G)')
disp('G= 10^(12.5/20)')
G= 10^(12.5/20)
disp('Kp=1/G')
Kp=1/G              % 0.2371     %[0,1]

pause

%   f.t. la con Kp
num_kp=num*Kp;
disp('f.t. lazo abierto con regulador proporcional')
G_la_p=tf(num_kp, den)
margin(G_la_p)

pause

%   f.t. lc con Kp
disp('f.t. lazo cerrado con regulador proporcional')
[num_lc_p, den_lc_p]= cloop(num_kp, den);
G_lc_p=tf(num_lc_p, den_lc_p)


pause

%   Respuesta a escalon con Kp
i=i+1;
figure(i)
plot(t,Y)       % tiempo
hold on;
step(G_lc_p, 'r')
axis([-2 8 -0.5 2])
title('Respuesta a escalon (en lazo cerrado) con Kp')
legend('Entrada escalón', 'Respuesta del sistema', 'Location', 'northeast')
hold off;

pause

%   Respuesta a rampa con Kp
i=i+1;
figure(i);
subplot(1,2,1)
plot(ramp,ramp)
hold on;
step(G_lc_p*R, 'r')
axis([0 100 0 100])
title('Respuesta a rampa (en lazo cerrado) con Kp')
legend('Entrada escalón', 'Respuesta del sistema', 'Location', 'northwest')
hold off;

subplot(1,2,2)
plot(ramp,ramp)
hold on;
step(G_lc_p*R, 'r')
axis([79.5 81.5 80 81])
title('Respuesta a rampa (en lazo cerrado) con Kp')
hold off;

%%
% Para extraer num. y denom. de una ft:
% [num_comp, den_comp]= tfdata(Gs_comp, 'v')
