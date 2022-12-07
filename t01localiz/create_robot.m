function [Robot] = create_robot(p0, totSteps, actSigma, senSigma, senRange)

%CREATE_ROBOT Crea un robot y asig
%   p0 es la pose inicial del robot
%   Prueba: 
%       lenFw=1; % longitud del paso de avance
%       totSteps=44; % número total de pasos
%       pos0 = [0 0 pi/2]';
%       p0=[0 0 pi]';lenFw=1;totSteps=44;Robot=create_robot(p0,lenFw, totSteps)
%
    Robot = {};
    Robot.xTrue = p0; % posición inicial real
    Robot.xOdom = p0; % posición odométrica
    Robot.xEst = p0; % posición estimada

    % Configuración LSE
    Robot.lseMaxIter = 100; % Máximo número de iteraciones en LSE
    Robot.lseTolerance = 1.0e-09; % Tolerancia sufiente para dentener LSE
    
    % Configuración FP
    Robot.fpParticles = 100; % número total de partículas
    Robot.fpPos = ones(3, Robot.fpParticles).*p0; % pose de partículas

    % Precisión de sensores y actuadores
    Robot.actSigma = actSigma;
    Robot.senSigma = senSigma;
    Robot.senRange = senRange;

    % Cálculo de errores
    Robot.errOdo = zeros(totSteps, 1); % error localización odométrica
    Robot.errLSE = zeros(totSteps, 1); % error estimación  LSE
    Robot.senLSE = zeros(totSteps, 1); % número de sensores visibles en LSE
    Robot.errFP  = zeros(totSteps, 2); % error est. FP posición/orientación
    Robot.senFP  = zeros(totSteps, 1); % número del sensor elegido en FP

end

