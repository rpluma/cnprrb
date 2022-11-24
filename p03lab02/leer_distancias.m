function z = leer_distancias(Mapa, Robot, Sensor)
%LEER_DISTANCIAS Lee la distancia a cada uno de los sensores en alcance
%   Robot.xTrue es la posición real del robot
%   Mapa contiene la lista de sensores
%   la lectura del sensor está sujeta a un ruido gaussiano con varianza var_d
%   los sensores a mayor distancia que el rango se descartan
%   los sensores fuera del FOV definido por +/-alfa también se descartan
%
%   num_sensores es el número de sensores en rango y orientación adecuada
%   h es un vector de distancias reales con ruido gaussiano
%   en el mapa cada sensor se colorea según si está en rango o no
    z = zeros(Mapa.nLandmarks, 1);
    for i = 1:Mapa.nLandmarks
        % calculamos la distancia real al landmark
        dist = norm(Robot.xTrue(1:2)-Mapa.Landmarks(1:2, i));

        % calculamos el ángulo que forman el robot y el landmark
        theta2=atan2(Mapa.Landmarks(2,i)-Robot.xTrue(2), ...
            Mapa.Landmarks(1,i)-Robot.xTrue(1));

        % calculamos la diferencia en grados entre la pose del robot y theta2
        diferencia = AngleWrap(theta2-Robot.xTrue(3))*180/pi;
        
        % si está en rango devolvemos la distancia con ruido gaussiano
        if (dist <= Sensor.rango) && (abs(diferencia) < Sensor.alpha)
            z(i) = abs(dist + Sensor.dStd*randn(1,1)); % distancia>0
            plot(Mapa.Landmarks(1,i),Mapa.Landmarks(2,i),'sg','LineWidth',4);
        else
            z(i) = -1; % descartar la distancia poniendo un valor negativo
            plot(Mapa.Landmarks(1,i),Mapa.Landmarks(2,i),'sr','LineWidth',4);
        end
    end
end

    