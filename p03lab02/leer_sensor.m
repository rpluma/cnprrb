function observacion = leer_sensor(Mapa, Robot)
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
    observacion=zeros(2, 1);

% 1 Selección aleatoria de un landmark
    eleccion = randperm(Mapa.nLandmarks, 1);
    landmark = Mapa.Landmarks(:, eleccion);

%2 cálculo de la distancia y ángulo
    observacion(1) = norm(landmark-Robot.xOdom(1:2));
    observacion(2) = atan2( ...
        landmark(2)-Robot.xOdom(2), landmark(1)-Robot.xOdom(1));
    %[2.1 observacion(1) observacion(2)*180/pi]
%3  afectación del ruido del sensor en base a matriz de covarianza
    ruido=sqrt(Robot.CovSensor)*rand(2,1);
    observacion = observacion+ruido;
    if (observacion(2) > pi)
        observacion(2) = observacion(2)-2*pi;
    elseif (observacion(2) <-pi)
        observacion(2) = observacion(2)+2*pi;
    end
    %[2.2 observacion(1) observacion(2)*180/pi]
end

