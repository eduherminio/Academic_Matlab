%%
% Author: 	Eduardo Cáceres de la Calle
% Subject: 	Control Systems Design
% Degree: 	Industrial Electronics and Automatic Control Engineering
% University:	Universidad de Valladolid (UVa) - EII
%
% Code written in 2016
% Uploaded for educational purposes only, don't be too hard on me :)


%Ejercicio 3
%Partimos del ejercicio 2
close all
clc
clear
% Funcion de transferencia con la que trabajaremos
num=[10];
den=[0.01, 0.25, 1, 0];
disp('Funcion de transferencia en lazo abierto')
G_la=tf(num, den)

Kp= 0.2371;

G_la_p= Kp*G_la

num=[25];
den=[0.01, 0.25, 1, 0];
disp('Funcion de transferencia en lazo abierto')
H_la=tf(num, den)
H_la_p= Kp*H_la

% Definiciones e inicializaciones
i=1;
t=[0:0.01:500];
t_h=-10:0.001:10;
Y=heaviside(t_h);

% Construcción de la f.t. de rampa
num_R=[1];                
den_R=[1, 0];
R=tf(num_R,den_R);  

% Señal de control
figure(i)
i=i+1
G_control= Kp/(1+Kp*G_la)
step(G_control)

% Cierre del lazo
    [num_lc, den_lc]= cloop(Kp*num, den);
    G_lc_Kp= tf(num_lc, den_lc)
%Comprobamos error estacionario nulo ante salto
figure(i)
i=i+1;
plot(t_h,Y)
hold on
step(G_lc_Kp,'r')
axis([-0.5 5 -0.5 2])
title('Respuesta del sistema realimentado ante escalón')
legend('Entrada escalón','Respuesta','Location','northeast')
hold off

pause

%Comprobamos margen de fase LA
figure(i)
i=i+1;
margin(G_la_p)

pause

%Comprobamos estabilidad del sistema de ganancia 25
figure(i)
i=i+1;
margin(H_la_p)

pause

%Comprobamos tiempo de respuesta LC
figure(i)
i=i+1;
plot(t_h,Y)
hold on
step(G_lc_Kp,'r')
axis([-0.5 8 -0.5 2])
title('Respuesta del sistema realimentado ante escalón')
legend('Entrada escalón','Respuesta','Location','northeast')
hold off


%Como el tiempo de respuesta es muy superior al  permitido, tenemos que
%reducirle. Para ello aumentamos AB aplicando comp adelanto
% Aplicamos compensador de adelanto
        
    %Diseñado apuntes estableciendo el angulo que queremos
        pause
        b_a= 57.695;
        num_comp_adelanto= [1 1.843134966];
        den_comp_adelanto= [1 106.3405576];
        
        disp('Compensador de adelanto')
        Comp_adelanto=b_a*tf(num_comp_adelanto, den_comp_adelanto)

        figure(i)
        i=i+1;
        margin(Comp_adelanto)
        
         disp('Funcion de transferencia con compensador y lazo cerrado')
        G_la_comp_adel=series(G_la_p, Comp_adelanto);
        
        %Señal de control
        figure(i)
        i=i+1;
   
        G_control=Kp*Comp_adelanto/(1+G_la_comp_adel)
        step(G_control)
        
        
    %Cerramos lazo y comprobamos respuesta

        figure(i)
        i=i+1;
        margin(G_la_comp_adel)
        
        [num_comp, den_comp]= tfdata(G_la_comp_adel, 'v');  % num y den
        [num_lc, den_lc]= cloop(num_comp, den_comp);
        G_lc_Kp_adelanto= tf(num_lc, den_lc)
        
        figure(i)
        i=i+1;
        plot(t_h,Y)
        hold on
        step(G_lc_Kp_adelanto,'r')
        hold off
        title('Respuesta con Kp y compensador ante escalon')
        legend('Entrada escalón','Respuesta','Location','northeast')
        axis([-0.5 8 -0.5 2])
        
   %subimos ganancia y realimentamos
   
        pause
        b_a= 162.23;
        num_comp_adelanto= [1 1.843134966];
        den_comp_adelanto= [1 106.3405576];
        
        disp('Compensador de adelanto')
        Comp_adelanto=b_a*tf(num_comp_adelanto, den_comp_adelanto)
        disp('Funcion de transferencia con compensador y lazo cerrado')
        G_la_comp_adel=series(G_la_p, Comp_adelanto);
        
        figure(i)
        i=i+1;
        margin(G_la_comp_adel)
        
        [num_comp, den_comp]= tfdata(G_la_comp_adel, 'v');  % num y den
        [num_lc, den_lc]= cloop(num_comp, den_comp);
        G_lc_Kp_adelanto= tf(num_lc, den_lc)
        
        figure(i)
        i=i+1;
        plot(t_h,Y)
        hold on
        step(G_lc_Kp_adelanto,'r')
        hold off
        legend('Entrada escalón','Respuesta con ganancia','Location','northeast')
        title('Respuesta con Kp y compensador ante escalon, con ganancia');
        axis([-0.5 8 -0.5 2])
        
        
   %Diseñado internet
        pause
        k=2.831391996;
        num_comp_adelanto= [0.5425538652 1];
        den_comp_adelanto= [0.009403749828 1];
        
        disp('Compensador de adelanto')
        Comp_adelanto=tf(num_comp_adelanto, den_comp_adelanto)

        figure(i)
        i=i+1;
        margin(Comp_adelanto)
        
    %Cerramos lazo y comprobamos respuesta
    
        
        disp('Funcion de transferencia con compensador y lazo cerrado')
        G_la_comp_adel=series(G_la_p, Comp_adelanto);
        
        figure(i)
        i=i+1;
        margin(G_la_comp_adel)
        
        pause
        
        G_la_comp_adel=k*G_la_comp_adel;
        figure(i)
        i=i+1;
        margin(G_la_comp_adel)
        
        [num_comp, den_comp]= tfdata(G_la_comp_adel, 'v');  % num y den
        [num_lc, den_lc]= cloop(num_comp, den_comp);
        G_lc_Kp_adelanto= tf(num_lc, den_lc)
        
        figure(i)
        i=i+1;
        plot(t_h,Y)
        hold on
        step(G_lc_Kp_adelanto,'r')
        hold off
        legend('Entrada escalón','Respuesta','Location','northeast')
        title('Compensador con ganancia procedimiento T alfa');
        axis([-0.5 8 -0.5 2])
        
  

