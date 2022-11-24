function [xEst] = estimar_posicion(Mapa, Robot, Sensor, Lse)
% num_sensores, z, xOdom, Mapa, tolerance, nIteration
%ESTIMAR_POSICION Estima la posición del robot en función de los sensores
%   num_sensores indica el número de sensores en rango
%   z son las lecturas de distancia a los sensores que incluyen ruido
%   xOdom es la posición del robot según la odometría
%   Mapa contiene las posiciones de los sensores

    % Sólo los sensores en rango tienen distancia positiva
    iValidSensors = find(Sensor.z >= 0); 
    numSensors = length(iValidSensors);

    %-------------------------------
    % 15 LSE - Least Square Estimation
    %-------------------------------
    xEst = Robot.xOdom; % inicializamos la estimación con la odometría
    iteration = 0;
    incr = ones(1,2);       % initialize increment (step)    
    h = zeros(numSensors, 1); % predicted observations (sensor model)
    jH = zeros(numSensors,2); % jacobian of the observation function for all the landmarks
    
    % utilizar sólo información de los landmarks en rango
    Landmarks=Mapa.Landmarks(:, iValidSensors);
    z = Sensor.z(iValidSensors, 1);
    
    % 16 LSE LOOP (iterative Method)
    while (norm(incr)>Lse.tolerance) && (iteration<Lse.max_iter)

        % 17 compute the prediction (from current xEst) and build the Jacobian
        for i = 1:numSensors
            Delta = Landmarks(1:2,i)-xEst(1:2);  % Expected distances between the map and the pose estimated
            h(i) = norm(Delta);            % predicted observation
            % Jacobian evaluated in xEst
            jH(i,1) = -Delta(1) / h(i);
            jH(i,2) = -Delta(2) / h(i);
        end
    
        % 18 Añadir R ruido dependiente z y calcular error=z-h
        R = diag(Sensor.dVar*sqrt(z));    % observation variance grow with the root of the distance
        error = z - h;              % difference between measurement and prediction

        % 19 incr!
        incr = inv(jH'*inv(R)*jH) * jH'*inv(R)*error;
    
        % 20 Update Estimation xEst
        xEst= xEst*1;
        incr = incr*1;

        xEst(1:2) = xEst(1:2)+incr;
        iteration = iteration+1;        
    end % while 
end

