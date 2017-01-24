%%
% Author: 	Eduardo Cáceres de la Calle
% Subject: 	Control Systems Design
% Degree: 	Industrial Electronics and Automatic Control Engineering
% University:	Universidad de Valladolid (UVa) - EII
%
% Code written in 2016
% Uploaded for educational purposes only, don't be too hard on me :)


%% 
clc
clear

%Definiciones
H.A=[-10 1; -0.1 -2]
H.B=[0; 4]
H.C=[1 0]
Tr=0.1
syms K1 K2
syms s

%Comprobacion de la controlabilidad
Co=[H.B H.A*H.B]
rank(Co)
pause
%%Como el rango de la matriz de controlabilidad es 2, el sistema es
%%controlable

%Requiere factor de amortiguamiento 1 y tiempo de respuesta 0.1s
%Factor de amotiguamiento 1-> críticamente amortiguado->(Ka)/(s+a)^2
%Rise time <=0.1 -> Tr=1.5/a -> a>=15
r=[-1.5/Tr, -1.5/Tr];
ds=poly(r)
%S=poly(0)
I=eye(2);
K=[K1 K2]


d=det(s*I-(H.A-H.B*K))
coef=coeffs(d);
pause

%Obtenemos las soluciones
K2=(ds(2)-coef(2))/coef(6)
K1=(ds(3)-coef(1)-K2*coef(4))/coef(5)
Q1=double(K1);
Q2=double(K2);

H.Q=[Q1 Q2]
d=det(s*I-(H.A-H.B*H.Q))
G=(1.5/Tr)^2;

[b,a]=ss2tf(H.A,H.B,H.C,0);

pause
simulink




