clc;
clear all;
close all;
%% 
% Creamos el robot

lenFw=1; % longitud del paso de avance
Size=10 ; % tamaño del cuadrado
totSteps=(Size+1)*4; % número total de pasos
Robot=create_robot(lenFw, totSteps);
%% 
% Creamos el mapa

nLandmarks=10; iFig=1; 
Mapa=create_map(Size, nLandmarks, iFig);
%% 
% Creamos el cuadrado

Robot.uOdom = Robot.uForward;
Robot.xTrue = [-Size/2, -Size/2, 0]';
Robot.xOdom = Robot.xTrue;
plot(Robot.xTrue(1),Robot.xTrue(2),'kx');   % Real
for i=1:4
    for j = 1:Size
        Robot.xTrue = comp_noisy(Robot.xTrue, Robot.uForward, Robot.actSigma);
        Robot.xOdom = comp_odom (Robot.xOdom, Robot.uForward);
        [zTrue, zNoisy, bVisible] = read_sensors(Robot, Mapa);
        if (sum(bVisible) >= 3)
            Robot.xEst = est_lse(Robot, Mapa, zNoisy, bVisible);
        else
            Robot.xEst = comp_odom(Robot.xEst, Robot.uOdom);
        end
        update_figure(Robot, Mapa, bVisible, 0);
        pause(0.5);
    end
    % giramos o avanzamos dependiendo si es esquina
    Robot.xTrue = comp_noisy(Robot.xTrue, Robot.uTurnLeft, Robot.actSigma);
    Robot.xOdom = comp_odom (Robot.xOdom, Robot.uTurnLeft);
    pause(2);
end