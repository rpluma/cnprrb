function [Robot] = create_robot(lenFw, totSteps)
%CREATE_ROBOT Crea un robot y asig
%   Prueba: 
%       lenFw=1; % longitud del paso de avance
%       totSteps=44; % número total de pasos
%       lenFw=1;totSteps=44;Robot=create_robot(lenFw, totSteps)
%
    Robot = {};
    Robot.xTrue = [0 0 0]'; % posición inicial real
    Robot.xOdom = Robot.xTrue; % posición odométrica
    Robot.xEst = Robot.xTrue; % posición estimada

    % Configuración LSE
    Robot.lseMaxIter = 100; % Máximo número de iteraciones en LSE
    Robot.lseTolerance = 1.0e-09; % Tolerancia sufiente para dentener LSE
    
    % Configuración FP
    Robot.fpParticles = 100; % número total de partículas
    Robot.fpPos = zeros(3, Robot.fpParticles); % poses de las partículas

    % Acciones de actualización
    Robot.uForward = [lenFw 0 0]'; % acción de avanzar
    Robot.uTurnLeft = [0 0 pi/2]'; % acción de giro a la izquierda

    % Precisión de actuadores
    Robot.actSigma = [0.05 0.05 2.5*pi/180]'; % desviación típica [m m rad]'

    % Precisión de sensores
    Robot.senSigma = [0.05 2.5*pi/180]'; % desviación típica en [m rad]'
    Robot.senRange = [20 90*pi/180]'; % alcance (distancia/FOV) [m rad]'

    % Cálculo de errores
    Robot.errOdo = zeros(totSteps, 1); % error localización odométrica
    Robot.errLSE = zeros(totSteps, 1); % error estimación  LSE
    Robot.senLSE = zeros(totSteps, 1); % número de sensores visibles en LSE
    Robot.errFP  = zeros(totSteps, 3); % error est. FP con los 3 métodos
    Robot.senFP  = zeros(totSteps, 1); % número del sensor elegido en FP

end

