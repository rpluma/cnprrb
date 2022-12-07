%% EJERCICIO PUNTUABLE - LSE
%% Enunciado
% Los sensores reales presentan ciertas limitaciones  físicas en cuanto al rango 
% y campo de visión. Modifica el ejercicio anterior para  contemplar que el sensor 
% sólo proporciona medidas en un rango limitado 𝑟𝑙 y una  orientación limitada 
% ±𝛼 con respecto a la pose del robot. Podría darse el caso que no  existieran 
% landmarks en el campo de visión del sensor, con lo que el robot no dispondría 
% de información sensorial en una iteración
% 
% Aumenta el número de Landmarks a 10 y  considera el caso particular de un 
% sensor con  un rango máximo de 20m de alcance, y un FOV de ±60º
% 
% Indica claramente en la memoria como tratas  el caso de no tener suficientes 
% observaciones,  y muéstralo en las gráficas de posición y de  error. Añade/explica 
% solo la parte del código  que has modificado
%% Inicialización del mapa
% Aunque el enunciado no lo especifica, creamos un mapa de 50x50 similar al 
% que aparece en la figura para que haya sensores fuera de rango tanto por el 
% ángulo como por la distancia.

clc; clear all; close all;
rng(5); % semilla para hacer el ejercicio reproducible
MapSize = 50; nLandmarks=10;  
Mapa=create_map(MapSize, nLandmarks);
%% Inicialización del robot
% Creamos el robot parametrizado por:
%% 
% * Presisión de actuadores (actSigma), utilizando el caso más impreciso para 
% el movimiento en ambos ejes y aumentando la desviación típica para el ángulo 
% (0.5 m en los ejes y 2.5º en el ángulo)
% * Precisión de sensores (senSigma), utilizando el mismo valor que el enunciado 
% para la distancia, pero aumentando la imprecisión del ángulo (0.05 m y 2.5º 
% de desviación típica)
% * Rango de los sensores: utilizando el valor pedido en el enunciado (20m y 
% 60º)

% precisión actuadores y sensores
actSigma = [0.5 0.5 2.5*pi/180]'; % desviación típica en [m m rad]
senSigma = [0.05 2.5*pi/180]'; % desviación típica en [m rad]'
senRange = [20 60*pi/180]'; % alcance (distancia/FOV) [m rad]'

Size=10 ; % tamaño del cuadrado
totSteps = (Size+1)*4; % número total de pasos
p0 = [Size/2, -Size/2, pi/2]'; % posición inicial del robot
totSteps=(Size+1)*4; % número total de pasos
Robot=create_robot(p0, totSteps, actSigma, senSigma, senRange);

%% Creación del cuadrado

% Acciones de actualización
uForward = [2 0 0]'; % acción de avanzar
uTurnLeft = [0 0 pi/2]'; % acción de giro a la izquierda
uNext = uForward;

[zTrue, zNoisy, bVisible] = read_sensors(Robot, Mapa);
update_figure(Robot, Mapa, bVisible, 0);

for i=1:totSteps
    % Actualización de las posiciones real y odométrica
    Robot.xTrue = comp_noisy(Robot.xTrue, uNext, Robot.actSigma);
    Robot.xOdom = comp_odom (Robot.xOdom, uNext);
    [zTrue, zNoisy, bVisible] = read_sensors(Robot, Mapa);

    % Actualización de estimación por LSE si hay 3 o más balizas
    if (sum(bVisible) >= 3)
        Robot.xEst = est_lse(Robot, Mapa, zNoisy, bVisible);
       
    % Actualización desde última posición estimada si hay menos de 3
    else
        Robot.xEst = comp_odom(Robot.xEst, uNext);
    end

    % actualización de figura, errores y de número de sensores visibles
    update_figure(Robot, Mapa, bVisible, 0);
    Robot.errOdo(i) = norm(Robot.xTrue(1:2)-Robot.xOdom(1:2));
    Robot.errLSE(i) = norm(Robot.xTrue(1:2)-Robot.xEst(1:2));
    Robot.senLSE(i) = sum(bVisible);

    % decisión de girar o avanzar en el siguiente paso
    if mod(i, Size+1) == 0
        uNext = uTurnLeft;
    else
        uNext = uForward;
    end    
    pause(0.5);    
end
%% Visualización de errores

figure(2); 
set(gcf,'Visible','on');

subplot(311); plot(Robot.errOdo,'r'); hold on; 
avg_odo_error = mean(Robot.errOdo);
line([1 size(Robot.errOdo,1)],[avg_odo_error avg_odo_error]); 
title('Odometric error'); ylabel('Error (m)');

subplot(312); 
plot(Robot.errLSE,'g'); hold on; 
avg_est_error = mean(Robot.errLSE);
line([1 size(Robot.errLSE,1)],[avg_est_error avg_est_error]); 
title('Position Errors with LSE localization'); ylabel('Error (m)');

subplot(313); 
plot(Robot.senLSE,'k'); hold on; 
line([1 size(Robot.senLSE,1)],[2.5 2.5]); 
title('Sensors in range'); ylabel('# Sensors');
%xlabel('Iteration Number');


%%