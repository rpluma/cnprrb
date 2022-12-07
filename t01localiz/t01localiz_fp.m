%% EJERCICIO PUNTUABLE - FP
%% Enunciado
% Una vez tengas implementado y funcionando  correctamente el filtro de partículas 
% del ejercicio anterior, vamos a corregir la pose del  robot. 
% 
% Sabemos que el robot se “desvía” de su trayectoria deseada (el cuadrado de  
% lado 10m), pero gracias a la localización del FP ya sabemos dónde estamos realmente. 
% 
% Implementa en este ejercicio, que cada 5 iteraciones se añade un “paso extra” 
% para  corregir la pose del robot y hacer que vuelva al cuadrado. 
% 
% a. Muestra inicialmente una captura en la que se aprecie el correcto funcionamiento 
% del filtro (sin correcciones). Muestra una gráfica del error  entre xTrue y 
% xEst a lo largo de las iteraciones, tanto error en posición como en  orientación. 
% Comenta los resultados. 
% 
% b. Luego añade otra captura donde se aprecien las correcciones cada 5 pasos  
% implementados en este ejercicio. En este caso, muestra las gráficas de error  
% entre xEst y xOdom (para que se aprecie que cada 5 pasos el error debe  hacerse 
% muy pequeño), tanto de posición como de orientación.  
% 
% c. MUY IMPORTANTE: Recuerda que no podemos usar xTrue en los cálculos de  
% esta corrección, tan solo xEst.
%% Inicialización del mapa
% Creamos un mapa de 50x50 igual al del ejercicio puntuable LSE; para ello usamos 
% la misma semilla (rng) de forma que podamos comparar ambos ejercicios

clc; clear all; close all;
rng(5); % semilla para hacer el ejercicio reproducible
MapSize = 50; nLandmarks=10;  
Mapa=create_map(MapSize, nLandmarks);

%% Inicialización del robot
% Creamos el robot parametrizado por:
%% 
% * Presisión de actuadores (actSigma), utilizando los mismos valores que el 
% ejercicio LSE
% * Precisión de sensores (senSigma), utilizando los mismos valores due el ejercicio 
% LSE
% * Rango de los sensores: utilizando un rango de 200m y +/- 180º para no que 
% todos los sensores estén en rango, ya que sólo se va a utilziar uno

% precisión actuadores y sensores
actSigma = [0.5 0.5 2.5*pi/180]'; % desviación típica en [m m rad]
senSigma = [0.05 2.5*pi/180]'; % desviación típica en [m rad]'
senRange = [200 pi]'; % alcance (distancia/FOV) [m rad]'

Size=10 ; % tamaño del cuadrado
totSteps = (Size+1)*4; % número total de pasos
p0 = [Size/2, -Size/2, pi/2]'; % posición inicial del robot
totSteps=(Size+1)*4; % número total de pasos
Robot=create_robot(p0, totSteps, actSigma, senSigma, senRange);
%% Cuadrado sin corregir posición

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
    
    % actualización de partículas
    Robot.fpPos = comp_noisy(Robot.fpPos, uNext, Robot.actSigma);

    [zTrue, zNoisy, bVisible] = read_sensors(Robot, Mapa);

    % estimamos la posición mediante filtro de partículas
    [xEst, Best, bVisible] = est_fp(Robot, Mapa, zNoisy, 1);
    Robot.xEst = xEst; % nueva posición del robot
    Robot.fpPos = Best; % nueva posición de partículas seleccionadas
    
    % actualización de figura, errores y de número de sensores visibles
    update_figure(Robot, Mapa, bVisible, 1);
    Robot.errOdo(i) = norm(Robot.xTrue(1:2)-Robot.xOdom(1:2));
    Robot.errFP(i,1) = norm(Robot.xTrue(1:2)-Robot.xEst(1:2)); % posición
    Robot.errFP(i,2) = angle_sum(Robot.xTrue(3)-Robot.xEst(3),0); % orientación
    Robot.senFP(i) = find(bVisible, 1); % sensor utilizado para estimar

    % decisión de girar o avanzar en el siguiente paso
    if mod(i, Size+1) == 0
        uNext = uTurnLeft;
    else
        uNext = uForward;
    end    
    pause(0.5);    
end
%% Visualización de errores sin correcciones

figure; 
set(gcf,'Visible','on');

subplot(311); plot(Robot.errOdo,'r'); hold on; 
avg_odo_error = mean(Robot.errOdo);
line([1 size(Robot.errOdo,1)],[avg_odo_error avg_odo_error]); 
title('Odometric error'); ylabel('Error (m)');

subplot(312); 
plot(Robot.errFP(:,1),'g'); hold on; 
avg_est_error = mean(Robot.errFP(:,1));
line([1 size(Robot.errFP,1)],[avg_est_error avg_est_error]); 
title('Position error with FP - no corrections'); ylabel('Error (m)');

subplot(313); 
plot(Robot.errFP(:,2)*180/pi,'g'); hold on; 
avg_est_error = angle_sum(mean(Robot.errFP(:,2)),0)*180/pi;
line([1 size(Robot.errFP,1)],[avg_est_error avg_est_error]); 
title('Orientation errors with FP - no corrections'); ylabel('Error (º)');
%% 
% 
%% Cuadrado corrigiendo cada 5 movimientos
% El algoritmo es similar al caso anterior, pero llevamos la cuenta de pasos 
% sin corregir, y cuando llegamos a 5 aplicamos la correción:
%% 
% * Giramos el robot y las partículas hacia la posición requerida (odométrica)
% * Movemos la distancia que separa la posición estimada y la posición requerida
% * Giramos otra vez para que lleguen al ángulo requerido

rng(5); % semilla para hacer el ejercicio reproducible
MapSize = 50; nLandmarks=10;  
Mapa=create_map(MapSize, nLandmarks);
Robot=create_robot(p0, totSteps, actSigma, senSigma, senRange);

% Número de pasos sin corregir la posición
iSinCorregir=0;

% Acciones de actualización
uForward = [2 0 0]'; % acción de avanzar
uTurnLeft = [0 0 pi/2]'; % acción de giro a la izquierda
uNext = uForward;

[zTrue, zNoisy, bVisible] = read_sensors(Robot, Mapa);
update_figure(Robot, Mapa, bVisible, 0);

for i=1:totSteps
    % Aumentamos el número de pasos sin corregir
    iSinCorregir = iSinCorregir + 1;

    % Actualización de las posiciones real y odométrica
    Robot.xTrue = comp_noisy(Robot.xTrue, uNext, Robot.actSigma);
    Robot.xOdom = comp_odom (Robot.xOdom, uNext);
    
    % actualización de partículas
    Robot.fpPos = comp_noisy(Robot.fpPos, uNext, Robot.actSigma);

    [zTrue, zNoisy, bVisible] = read_sensors(Robot, Mapa);

    % estimamos la posición mediante filtro de partículas
    [xEst, Best, bVisible] = est_fp(Robot, Mapa, zNoisy, 1);
    Robot.xEst = xEst; % nueva posición del robot
    Robot.fpPos = Best; % nueva posición de partículas seleccionadas
    
    if (iSinCorregir >=5) % girar, avanzar y volver a girar
        % Diferencia entre pose estimada del robot y la que debería tener
        %[Robot.xOdom(1), Robot.xOdom(2), Robot.xOdom(3)*180/pi]
        %[Robot.xEst(1), Robot.xEst(2), Robot.xEst(3)*180/pi]

        delta = dist_angle(Robot.xEst, Robot.xOdom);
        %[delta(1) delta(2)*180/pi]
        update_figure(Robot, Mapa, bVisible, 1);
        line([Robot.xEst(1), Robot.xOdom(1)], [Robot.xEst(2), Robot.xOdom(2)],'LineWidth',4);
        iSinCorregir = 0;
                    
        % giro del robot y partículas hacia la posición odométrica
        u_giro1 = [0 0 delta(2)]'; % correción ángulo hacia odométrica
        Robot.xTrue = comp_noisy(Robot.xTrue, u_giro1, Robot.actSigma);
        Robot.fpPos = comp_noisy(Robot.fpPos, u_giro1, Robot.actSigma);
        Robot.xEst = comp_odom(Robot.xEst, u_giro1);
        
        % avance hasta la posición odométrica
        u_avanc = [delta(1) 0 0]'; % avance hasta odométrica
        Robot.xTrue = comp_noisy(Robot.xTrue, u_avanc, Robot.actSigma);
        Robot.fpPos = comp_noisy(Robot.fpPos, u_avanc, Robot.actSigma);
        Robot.xEst = comp_odom(Robot.xEst, u_avanc);
        
            
        % giro hacia la pose odométrica
        u_giro2 = [0 0 Robot.xOdom(3)-Robot.xEst(3)]';
        Robot.xTrue = comp_noisy(Robot.xTrue, u_giro2, Robot.actSigma);
        Robot.fpPos = comp_noisy(Robot.fpPos, u_giro2, Robot.actSigma);
        Robot.xEst = comp_odom(Robot.xEst, u_giro2);
        
        % estimamos la posición mediante filtro de partículas
        [xEst, Best, bVisible] = est_fp(Robot, Mapa, zNoisy, 1);
        Robot.xEst = xEst; % nueva posición del robot
        Robot.fpPos = Best; % nueva posición de partículas seleccionadas
    end

    % actualización de figura, errores y de número de sensores visibles
    update_figure(Robot, Mapa, bVisible, 1);
    Robot.errOdo(i) = norm(Robot.xTrue(1:2)-Robot.xOdom(1:2));
    Robot.errFP(i,1) = norm(Robot.xTrue(1:2)-Robot.xEst(1:2)); % posición
    Robot.errFP(i,2) = angle_sum(Robot.xTrue(3)-Robot.xEst(3),0); % orientación
    Robot.senFP(i) = find(bVisible, 1); % sensor utilizado para estimar


    % decisión de girar o avanzar en el siguiente paso
    if mod(i, Size+1) == 0
        uNext = uTurnLeft;
    else
        uNext = uForward;
    end    
    pause(0.5);    
end
%% Visualización de errores con correcciones

figure; 
set(gcf,'Visible','on');

subplot(311); plot(Robot.errOdo,'r'); hold on; 
avg_odo_error = mean(Robot.errOdo);
line([1 size(Robot.errOdo,1)],[avg_odo_error avg_odo_error]); 
title('Odometric error'); ylabel('Error (m)');

subplot(312); 
plot(Robot.errFP(:,1),'g'); hold on; 
avg_est_error = mean(Robot.errFP(:,1));
line([1 size(Robot.errFP,1)],[avg_est_error avg_est_error]); 
title('Position errors with FP - Periodic corrections'); ylabel('Error (m)');

subplot(313); 
plot(Robot.errFP(:,2)*180/pi,'g'); hold on; 
avg_est_error = angle_sum(mean(Robot.errFP(:,2)),0)*180/pi;
line([1 size(Robot.errFP,1)],[avg_est_error avg_est_error]); 
title('Orientation errors with FP - Periodic corrections'); ylabel('Error (º)');
%% 
%