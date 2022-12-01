function [zTrue, landmark] = leer_sensor(Mapa, Robot)
%LEER_SENSOR Summary of this function goes here
%   Mapa: mapa de landmarks 
%   Robot: información sobre el robot incluyendo su pose
%   
%   1) selecciona aleatoriamente un solo landmark de entre todos los  disponibles
%   2) cálculo de distancia y ángulo entre el robot y el landmark
%   3) afectación del ruido del sensor en base a matriz de covarianza
%
%   Devuelve la observación (distancia y ángulo) del sensor a dicho landmark

% 0 Inicialización
    zTrue=zeros(2, 1);

% 1 Selección aleatoria de un landmark
    eleccion = randperm(Mapa.nLandmarks, 1);
    landmark = Mapa.Landmarks(:, eleccion);

%2 cálculo de la distancia y ángulo desde xTrue
    zTrue = dist_ang(Robot.xTrue, landmark);

%3  afectación del ruido del sensor en base a matriz de covarianza
    ruido=sqrt(Robot.sensorCov)*rand(2,1);
    zTrue = zTrue+ruido;
    zTrue(2) = AngleWrap(zTrue(2));
end

