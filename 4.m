%%
% Author: 	Eduardo Cáceres de la Calle
% Subject: 	Control Systems Design
% Degree: 	Industrial Electronics and Automatic Control Engineering
% University:	Universidad de Valladolid (UVa) - EII
%
% Code written in 2016
% Uploaded for educational purposes only, don't be too hard on me :)


%Ejercicio 4
close all
clear
clc

% Funcion de transferencia con la que trabajaremos
num=[10];
den=[0.01, 0.25, 1, 0];
Ts=0.01;
G_cont_la=tf(num,den)
disp('Funcion de transferencia en lazo abierto')
[Nz,Dz]=c2dm(num, den, Ts,'zoh')
G_disc_la=tf(Nz,Dz,Ts)

i=1;
t=[0:0.01:500];
t_h=-10:0.01:100;
Y=heaviside(t_h);

%Respuesta ante escalón

figure(i)
i=i+1;
t=[0:0.01:500];
t_h=-10:0.001:100;
Y=heaviside(t_h);
plot(t_h,Y)
hold on
step(G_disc_la,'r')
title('Respuesta a escalon en LA')
legend('Entrada escalón','Respuesta','Location', 'northwest')
axis([-1 10 -1 100])
hold off

    %Cerramos el lazo
    [num_tf, den_tf]= tfdata(G_disc_la, 'v');
    [num_lc, den_lc]= cloop(num_tf,den_tf);
    G_disc_lc=tf(num_lc,den_lc,Ts)
    
    %Respuesta
    figure(i)
    i=i+1;
    plot(t_h,Y)
    hold on
    step(G_disc_lc,'r')
    title('Respuesta a escalon en LC')
    legend('Entrada escalon','Respuesta','Location', 'northwest')
    axis([-1 15 -0.5 2])
    hold off
    
    
%Margen de fase
figure(i)
i=i+1;
margin(G_disc_la)

%Aplicamos un proporcional al sistema para aumentar el margen de fase
    Kp=0.2818382931
    G_disc_la_Kp=Kp*G_disc_la;
    
    figure(i)
    i=i+1;
    margin(G_disc_la_Kp)

%Estabilidad del sistema con ganancia 70
    %Aumentamos la ganancia del sistema y comprobamos el margen
    Gain=7;
    H_disc_la_Kp=Gain*G_disc_la_Kp
    
    figure(i)
    i=i+1;
    margin(H_disc_la_Kp)
    %El sistema esta en el limite de la estabilidad, del lado de la misma.
   
    
%Tiempo de respuesta de 0.5s, sobre sistema de ganancia 10 con margen 55
    %Cerramos el lazo
    [num_tf, den_tf]= tfdata(G_disc_la_Kp, 'v');
    [num_lc, den_lc]= cloop(num_tf,den_tf);
    G_disc_lc_Kp=tf(num_lc,den_lc,Ts)
    
    %Respuesta
    figure(i)
    i=i+1;
    plot(t_h,Y)
    hold on
    step(G_disc_lc_Kp,'r')
    title('Respuesta a escalon en LC')
    legend('Entrada escalón','Respuesta','Location', 'northwest')
    axis([-1 30 -1 3])
    hold off

    %Cumple con la condicion.
    
%Tiempo de respuesta de 0.5s, sobre sistema de ganancia 70 
    %Cerramos el lazo
    [num_tf, den_tf]= tfdata(H_disc_la_Kp, 'v');
    [num_lc, den_lc]= cloop(num_tf,den_tf);
    H_disc_lc_Kp=tf(num_lc,den_lc,Ts)
    
    %Respuesta
    figure(i)
    i=i+1;
    plot(t_h,Y)
    hold on
    step(H_disc_lc_Kp,'r')
    title('Respuesta a escalon en LC ganancia 70')
    legend('Entrada escalón','Respuesta','Location', 'northwest')
    axis([-1 30 -0.5 2])
    hold off
    
    %Respuesta ante rampa
    numr=[1];
    denr=[1 0];
    [Nzr,Dzr]=c2dm(numr, denr, Ts,'zoh')
    Rampa=tf(Nzr,Dzr,Ts)
        
        %Respuesta
        G_rampa_dlc=series(Rampa, G_disc_lc_Kp);
        figure(i)
        i=i+1;
        plot(t_h,t_h)
        hold on
        step(G_rampa_dlc,'r')
        axis([0 20 -1 20])
        legend('Entrada rampa','Respuesta','Location', 'northwest')
        title('Respuesta ante rampa')
        hold off
  
    %Reducir el error ante rampa a lo menor posible
    %Compensador de atraso
    numc=[1 0.2];
    denc=[1 0.01];
    [Nzr,Dzr]=c2dm(numc, denc, Ts,'zoh')
    Comp_atras=tf(Nzr,Dzr,Ts)
    
    figure(i)
    i=i+1;
    margin(Comp_atras)
    
    G_comp=series(G_disc_la_Kp,Comp_atras)
    
    % Señal de control
    figure(i)
    i=i+1;
    G_Control=(Comp_atras*Kp)/(1+G_comp)
    step(G_Control)
    
        %Cerrado de lazo
        G_comp=series(G_disc_la_Kp,Comp_atras)
        
        figure(i)
        i=i+1;
        margin(G_comp)
        
        [num_tf, den_tf]= tfdata(G_comp, 'v');
        [num_c, den_c]= cloop(num_tf,den_tf);
        G_comp_lc_Kp=tf(num_c,den_c,Ts)
        
        %Respuesta
        G_comp_ramp=series(Rampa, G_comp_lc_Kp)
        figure(i)
        i=i+1;
        subplot(2,1,1)
        plot(t_h,t_h)
        hold on
        step(G_comp_ramp,'r')
        hold on
        step(G_rampa_dlc,'y')
        axis([0 100 -1 100])
        title('Respuesta ante rampa con compensador atraso')
        legend('Entrada rampa','Respuesta con compensador','Respuesta sin compensador','Location', 'northwest')
        hold off
        subplot(2,1,2)
        plot(t_h,t_h)
        hold on
        step(G_comp_ramp,'r')
        hold on
        step(G_rampa_dlc,'y')
        axis([51.6 51.7 51.1 51.75])
        title('Respuesta ante rampa con compensador atraso')
        legend('Entrada rampa','Respuesta con compensador','Respuesta sin compensador','Location', 'northwest')
        hold off
    %Respuesta ante rampa ganancia 70
    numr=[1];
    denr=[1 0];
    [Nzr,Dzr]=c2dm(numr, denr, Ts,'zoh')
    Rampa=tf(Nzr,Dzr,Ts)
        
        %Respuesta
        H_rampa_dlc=series(Rampa, H_disc_lc_Kp);
        figure(i)
        i=i+1;
        plot(t_h,t_h)
        hold on
        step(H_rampa_dlc,'r')
        axis([0 30 -1 30])
        legend('Entrada rampa','Respuesta','Location', 'northwest')
        title('Respuesta ante rampa con ganancia 70')
        hold off
  
    %Reducir el error ante rampa a lo menor posible ganacia 70
    %Compensador de atraso
    numc=[1 0.2];
    denc=[1 0.01];
    [Nzr,Dzr]=c2dm(numc, denc, Ts,'zoh')
    Comp_atras=tf(Nzr,Dzr,Ts)
    
        %Cerrado de lazo
        H_comp=series(H_disc_la_Kp,Comp_atras)
        [num_tf, den_tf]= tfdata(H_comp, 'v');
        [num_c, den_c]= cloop(num_tf,den_tf);
        H_comp_lc_Kp=tf(num_c,den_c,Ts)
        
        %Respuesta
        H_comp_ramp=series(Rampa, H_comp_lc_Kp)
        figure(i)
        i=i+1;
        subplot(2,1,1)
        plot(t_h,t_h)
        hold on
        step(H_comp_ramp,'r')
        hold on
        step(H_rampa_dlc,'y')
        axis([0 100 -1 100])
        title('Respuesta ante rampa con ganancia 70 y compensador atraso')
        legend('Entrada rampa','Respuesta con compensador','Respuesta sin compensador','Location', 'northwest')
        hold off
        subplot(2,1,2)
        plot(t_h,t_h)
        hold on
        step(H_comp_ramp,'r')
        hold on
        step(H_rampa_dlc,'y')
        axis([51.6 51.7 51.55 51.75])
        title('Respuesta ante rampa con compensador atraso')
        legend('Entrada rampa','Respuesta con compensador','Respuesta sin compensador','Location', 'northwest')
        hold off
    
        
