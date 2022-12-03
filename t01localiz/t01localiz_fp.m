clc;
clear all;
close all;
%% 
% Creamos el robot y el mapa

rng(1); % semilla para que los mapas sean iguales

% robot
lenFw=1; % longitud del paso de avance
Size=10 ; % tamaño del cuadrado
p0 = [-Size/2, -Size/2, 0]';
totSteps=(Size+1)*4; % número total de pasos
Robot=create_robot(p0, lenFw, totSteps);

% mapa
nLandmarks=10; 
[Mapa, f1]=create_map(Size, nLandmarks);
plot(Robot.xTrue(1),Robot.xTrue(2),'kx');   % Real

iMethod = 1;
        
for i=1:4
    for j = 1:Size
        Robot.xTrue = comp_noisy(Robot.xTrue, Robot.uForward, Robot.actSigma);
        Robot.xOdom = comp_odom (Robot.xOdom, Robot.uForward);
        % movemos las partículas hacia adelante
        Robot.fpPos = comp_noisy(Robot.fpPos, Robot.uForward, Robot.actSigma);
        
        [zTrue, zNoisy, bVisible] = read_sensors(Robot, Mapa);
        
        % estimamos la posición mediante filtro de partículas
        [xEst, Best, bVisible] = est_fp(Robot, Mapa, zNoisy, iMethod);
        Robot.xEst = xEst;
        Robot.fpPos = Best;

        % visualizamos incluyendo las partículas (último argumento=1)
        update_figure(Robot, Mapa, bVisible, 1);
        pause(0.5);
    end
    % giramos o avanzamos dependiendo si es esquina
    Robot.xTrue = comp_noisy(Robot.xTrue, Robot.uTurnLeft, Robot.actSigma);
    Robot.xOdom = comp_odom (Robot.xOdom, Robot.uTurnLeft);
    
    % giramos las partículas a la izquierda
    Robot.fpPos = comp_noisy(Robot.fpPos, Robot.uTurnLeft, Robot.actSigma);
    [zTrue, zNoisy, bVisible] = read_sensors(Robot, Mapa);
    [xEst, BestParticles] = est_fp(Robot, Mapa, zNoisy, iMethod);
    Robot.xEst = xEst;
    Robot.fpPos = BestParticles;
        
    pause(1);
end
%% 
% Creamos el cuadrado con correcciones

rng(1); % semilla para que los mapas sean iguales

% robot
lenFw=1; % longitud del paso de avance
Size=10 ; % tamaño del cuadrado
p0 = [-Size/2, -Size/2, 0]';
totSteps=(Size+1)*4; % número total de pasos
Robot=create_robot(p0, lenFw, totSteps);

% mapa
nLandmarks=10; 
[Mapa, f2]=create_map(Size, nLandmarks);
plot(Robot.xTrue(1),Robot.xTrue(2),'kx');   % Real

iMethod = 1;
iSinCorregir = 0;    
        
for i=1:4
    for j = 1:Size
        iSinCorregir = iSinCorregir + 1;
        Robot.xTrue = comp_noisy(Robot.xTrue, Robot.uForward, Robot.actSigma);
        Robot.xOdom = comp_odom (Robot.xOdom, Robot.uForward);
        % movemos las partículas hacia adelante
        Robot.fpPos = comp_noisy(Robot.fpPos, Robot.uForward, Robot.actSigma);
        
        [zTrue, zNoisy, bVisible] = read_sensors(Robot, Mapa);
        
        % estimamos la posición mediante filtro de partículas
        [xEst, Best, bVisible] = est_fp(Robot, Mapa, zNoisy, iMethod);
        Robot.xEst = xEst;
        Robot.fpPos = Best;

        % visualizamos incluyendo las partículas (último argumento=1)
        update_figure(Robot, Mapa, bVisible, 1);
        pause(0.5);

        if (iSinCorregir >=5) % girar, avanzar y volver a girar
            
            iSinCorregir = 0;
            delta = dist_angle(Robot.xEst, Robot.xOdom);
            
            % giro del robot hacia la posición odométrica
            u_giro1 = [0 0 delta(2)]'
            Robot.xTrue = comp_noisy(Robot.xTrue, u_giro1, Robot.actSigma);
            Robot.xEst  = comp_noisy(Robot.xEst,  u_giro1, Robot.actSigma);
            Robot.fpPos = comp_noisy(Robot.fpPos, u_giro1, Robot.actSigma);
            
            % avance hasta la posición odométrica
            u_avanc = [delta(1) 0 0]'
            Robot.xTrue = comp_noisy(Robot.xTrue, u_avanc, Robot.actSigma);
            Robot.xEst  = comp_noisy(Robot.xEst,  u_avanc, Robot.actSigma);
            Robot.fpPos = comp_noisy(Robot.fpPos, u_avanc, Robot.actSigma);
            
            % giro hacia la posición odométrica
            u_giro2 = [0 0 Robot.xOdom(3)-Robot.xEst(3)]'
            Robot.xTrue = comp_noisy(Robot.xTrue, u_giro2, Robot.actSigma);
            Robot.xEst  = comp_noisy(Robot.xEst,  u_giro2, Robot.actSigma);            
            Robot.fpPos = comp_noisy(Robot.fpPos, u_giro2, Robot.actSigma);
        end
    end
    % giramos o avanzamos dependiendo si es esquina
    Robot.xTrue = comp_noisy(Robot.xTrue, Robot.uTurnLeft, Robot.actSigma);
    Robot.xOdom = comp_odom (Robot.xOdom, Robot.uTurnLeft);
    
    % giramos las partículas a la izquierda
    Robot.fpPos = comp_noisy(Robot.fpPos, Robot.uTurnLeft, Robot.actSigma);
    [zTrue, zNoisy, bVisible] = read_sensors(Robot, Mapa);
    [xEst, BestParticles] = est_fp(Robot, Mapa, zNoisy, iMethod);
    Robot.xEst = xEst;
    Robot.fpPos = BestParticles;
        
    pause(1);
end

%% 
% Cuadrado con correciones
% 
% 

        
%% 
% for i=1:4
% 
% for j = 1:Size
% 
% Robot.xTrue = comp_noisy(Robot.xTrue, Robot.uForward, Robot.actSigma);
% 
% Robot.xOdom = comp_odom (Robot.xOdom, Robot.uForward);
% 
% % movemos las partículas hacia adelante
% 
% Robot.fpPos = comp_noisy(Robot.fpPos, Robot.uForward, Robot.actSigma);
% 
% 
% 
% [zTrue, zNoisy, bVisible] = read_sensors(Robot, Mapa);
% 
% 
% 
% % estimamos la posición mediante filtro de partículas
% 
% [xEst, Best, bVisible] = est_fp(Robot, Mapa, zNoisy, iMethod);
% 
% Robot.xEst = xEst;
% 
% Robot.fpPos = Best;
% 
% 
% 
% % visualizamos incluyendo las partículas (último argumento=1)
% 
% update_figure(Robot, Mapa, bVisible, 1);
% 
% pause(0.5);
% 
% end
% 
% % giramos o avanzamos dependiendo si es esquina
% 
% Robot.xTrue = comp_noisy(Robot.xTrue, Robot.uTurnLeft, Robot.actSigma);
% 
% Robot.xOdom = comp_odom (Robot.xOdom, Robot.uTurnLeft);
% 
% 
% 
% % giramos las partículas a la izquierda
% 
% Robot.fpPos = comp_noisy(Robot.fpPos, Robot.uTurnLeft, Robot.actSigma);
% 
% [zTrue, zNoisy, bVisible] = read_sensors(Robot, Mapa);
% 
% [xEst, BestParticles] = est_fp(Robot, Mapa, zNoisy, iMethod);
% 
% Robot.xEst = xEst;
% 
% Robot.fpPos = BestParticles;
% 
% 
% 
% pause(2);
% 
% end