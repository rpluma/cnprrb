function [xEst, numSensors] = est_lse(Robot, Mapa, zNoisy, bVisible)
%EST_LSE Estimación de la posición con Least Squared Errors
%   zNoisy es una matriz de 2xn con distancia y ángulo a cada landmark
%   vVisible es un vector que indica los sensores que están visibles
%   pruebas
%       xEst = est_lse(Robot, Mapa, zNoisy, bVisible)

    iValidSensors = find(bVisible == 1); % sólo usar sensores en rango
    numSensors = length(iValidSensors); % número de sensores en rango

    xEst = Robot.xOdom; % inicializamos la estimación con la odometría
    iteration = 0;
    incr = ones(1,2);       % initialize increment (step)    
    h = zeros(numSensors, 1); % predicted observations (sensor model)
    jH = zeros(numSensors,2); % jacobian of the observation function for all the landmarks
    
    % utilizar sólo información de los landmarks en rango
    Landmarks=Mapa.Landmarks(:, iValidSensors);
    zd = zNoisy(1, iValidSensors); % usar sólo sensor de distancia
    zd = zd'; % trasponer para que haya una lectura por fila
    
    % LSE LOOP (iterative Method)
    while (norm(incr)>Robot.lseTolerance) && (iteration<Robot.lseMaxIter)
        iteration = iteration+1;        

        % compute the prediction (from current xEst) and build the Jacobian
        for i = 1:numSensors
            % Expected distances between the map and the pose estimated
            Delta = Landmarks(1:2,i)-xEst(1:2);  
            h(i) = norm(Delta);            % predicted observation
            % Jacobian evaluated in xEst
            jH(i,1) = -Delta(1) / h(i);
            jH(i,2) = -Delta(2) / h(i);
        end
    
        % Para cada landmark, distancia leída vs distancia estimada
        error = zd - h;   % difference between measurement and prediction

        % observation variance grow with the root of the distance
        R = diag(Robot.senSigma(1)^2*sqrt(zd));
        
        % incr!
        incr = inv(jH'*inv(R)*jH) * jH'*inv(R)*error
    
        % Update Estimation xEst
        xEst(1:2) = xEst(1:2)+incr

    end % while 
end

