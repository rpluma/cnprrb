function [zTrue, zNoisy, bVisible] = read_sensors(Robot,Mapa)
%LEER_SENSORES Lee los sensores desde la posición real añadiendo ruido
%   zTrue devuelve la distancia y ángulo a cada sensor
%   zNoisy añade ruido a zTrue
%   bVisible indica si el sensor está dentro de la distancia y rango FOV
%   Se añade ruido a la lectura que depende de senSigma
%   pruebas
%       [zTrue, zNoisy, bVisible] = read_sensors(Robot, Mapa)

    zTrue = zeros(2, Mapa.nLandmarks);
    zNoisy = zeros(2, Mapa.nLandmarks);
    bVisible = zeros(1, Mapa.nLandmarks);

    for i = 1:Mapa.nLandmarks
        zTrue(:, i) = dist_angle(Robot.xTrue, Mapa.Landmarks(:, i));
        
        % TODO CONFIRMAR QUE AQUÍ NO DEPENDE DE LA DISTANCIA
        zNoisy(:, i) = zTrue(:,i) + Robot.senSigma .*randn(2, 1);
        % el ruido de la distancia es proporcional a su raiz cuadrada
        %zNoisy(1,i) = zTrue(1:i)+Robot.senSigma(1)*randn(1,1)*sqrt(zTrue(1:i));
        % el ruido del ángulo no depende de la diferencia de ángulos
        %zNoisy(2,i) = zTrue(2:i)+Robot.senSigma(2)*randn(1,1);
        
        % solo son visibles los sensores en rango de distancia y ángulo
        bVisible(i) = ...
            (zTrue(1,i) <= Robot.senRange(1)) && ... % distancia < rango
            (abs(zTrue(2,i)) <= Robot.senRange(2)); % angulo < FOV            
    end
end

