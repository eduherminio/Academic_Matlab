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

% Funcion de transferencia con la que trabajaremos
num=[10];
den=[0.01, 0.25, 1, 0];
disp('Funcion de transferencia en lazo abierto')
G_la=tf(num, den)

% Definiciones e inicializaciones
i=1;
t=[0:0.01:500];
t_h=-10:0.001:10;
Y=heaviside(t_h);

num_R=[1];                % Construcción de la f.t. de rampa
den_R=[1, 0];
R=tf(num_R,den_R);  


% Del ejercicio anterior obtuvimos:
Kp= 0.2371;

% **Ejercicio 2**

% Estudio de la estabilidad con ganancia € (10,25)
pause
figure(i)
num_max=25;
G_25=tf(num_max, den)
margin(G_25)

pause

i=i+1;
figure(i)
num_max_p=25*Kp;
disp('Aplicando el proporcional Kp=0.2371 ')
G_25_p=tf(num_max_p, den);
margin(G_25_p)
disp('MG siempre positivo: estabilidad aunque la ganancia aumento de 10 a 25')
disp('Esto podría haberse logrado con cualquier otra Kp (>0)')

pause

disp('f.t. la con Kp')
G_la_p= Kp*G_la

% Para ver márgenes de fase y ganancia, representar bode:
i=i+1;
figure(i)
subplot(1,2,1)  % f.t. la sin Kp
margin(G_la)
subplot(1,2,2)  %f.t. la con Kp
margin(G_la_p)

% Aplicamos compensador de atraso
for cont= 1:2
    pause
    
    if cont==1
           % Primer intento de compensador
        num_comp= [1 0.01];
        den_comp= [1 0.003];    
    else
           % Compensador con pérdida de MF aceptable y máxima reducción del error
        num_comp= [1 0.2];
        den_comp= [1 0.06];
    end
    
    % t.f. del compensador
    Comp=tf(num_comp, den_comp)
    i=i+1;
    figure(i)
    bode(Comp)
    
    pause
    
    % t.f. ya compensada
    disp('t.f. con proporcional y compensador, la')
    G_comp= series(Comp, G_la_p)   % multiplicacion
    i=i+1;
    figure(i)
    margin(G_comp)
    
    pause
    
    % Cierre del lazo
    [num_comp, den_comp]= tfdata(G_comp, 'v');  % num y den
    [num_lc, den_lc]= cloop(num_comp, den_comp);
    disp('t.f. com proporcional y compensador, lc')
    G_lc_Kp_comp= tf(num_lc, den_lc)
    
    % Respuesta ante entrada escalón
    if cont==1
        i=i+1;
        figure(i);
        subplot(2,1,1)
        plot(t_h,Y)
        hold on
        step(G_lc_Kp_comp, 'r')   %Respuesta a entrada escalón
        title('Respuesta con Kp y compensador ante escalon');
        axis([-2 30 -0.25 1.25])
        legend('Entrada escalón','Respuesta del sistema','Location','southeast')
        hold off
        
        subplot(2,1,2)
        t_aux=[0:0.01:100];
        Y_aux=heaviside(t_aux);
        plot(t_aux,Y_aux)
        hold on
        step(G_lc_Kp_comp, 'r')   %Respuesta a entrada escalón
        title('Respuesta con Kp y compensador ante escalon');
        axis([30 50 +0.75 1.25])
        legend('Entrada escalón','Respuesta del sistema','Location','southeast')
        hold off
        
    end
    
    pause
   
    % Respuestas ante entrada rampa
    [num_kp, den_kp]= tfdata(G_la_p, 'v');
    [num_sin, den_sin]= cloop(num_kp, den_kp);   % num_kp= Kp*nu,
    G_sin=tf(num_sin, den_sin)

    i=i+1;
    figure(i);
    subplot(1,2,1)
    
    step(G_lc_Kp_comp*R, 'r')    % Respuesta a entreda rampa con Kp y compensador de atraso
    
    hold on
    step(R*G_sin, 'c')          % Respuesta a entrada ranpa con Kp pero sin compensador de atraso

    hold on
    plot(t, t, 'y')             % Entrada rampa

    axis([0 100 0 100])
    if cont==1
        title('Ramp response con  el primer compensador')
    else
        title('Ramp response con  el segundo compensador')
    end 
    hold off
    
    
    subplot(1,2,2)

    step(G_lc_Kp_comp*R, 'r')    % Respuesta a entreda rampa con Kp y compensador de atraso
    
    hold on
    step(R*G_sin, 'c')          % Respuesta a entrada rampa con Kp pero sin compensador de atraso

    hold on
    plot(t, t, 'y')             % Entrada rampa

    axis([80 82 80 81])
    if cont==1
        title('Ramp response con  el primer compensador')
    else
        title('Ramp response con  el segundo compensador')
    end
    legend('Respuesta con compensador','Respuesta sin compensador','Entrada rampa','Location','southeast')
    hold off

end
