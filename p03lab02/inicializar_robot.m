function [Robot] = inicializar_robot(Mapa, square_side)
%INICIALIZAR_ROBOT inicializa el robot y lo dibuja en el mapa


    Robot = {};
    Robot.xTrue = [-square_side/2 -square_side/2 0]'; % abajo a la izquierda
    Robot.xOdom = Robot.xTrue;
    Robot.xEst = Robot.xTrue;
    %Robot.sensorStd = [0.5 0.5*pi/180]; % desviación típica del sensor
    Robot.sensorStd = [0.11 0.5*pi/180]; % 1 cm de desviación y medio grado
    %Robot.sensorCov = diag(Robot.sensorStd.^2); % matriz covarianza
    Robot.sensorCov = diag(Robot.sensorStd); % matriz covarianza
    Robot.sensorCovInv = inv(Robot.sensorCov);
    plot(Robot.xTrue(1),Robot.xTrue(2),'kx');   % Real Position

    % preparar la visualización
    close all;
    figure(1);
    set(gcf,'Visible','on');    % pup-up window
    hold on; 
    grid on; 
    axis equal;

    % plot landmarks (magenta squares)
    plot(Landmarks(1,:),Landmarks(2,:),'sm','LineWidth',4);

    % Plot Labels
    for k = 1:Mapa.nLandmarks
        text( Landmarks(1,k)+1,Landmarks(2,k), sprintf('L%d',k) );
    end

end

