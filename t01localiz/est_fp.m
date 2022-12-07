function [xEst, Best, bVisible] = est_fp(Robot, Mapa, zNoisy, iMethod)
%EST_FP Estimar la posición del robot mediante filtro de partículas
%   zNoisy tiene las lecturas de todos los sensores; elegimos uno al azar
%   iMethod es el método de estimación elegido

    % elegimos un sensor al azar
    iSelected = randperm(Mapa.nLandmarks, 1);
    z = zNoisy(: , iSelected);
    landmark = Mapa.Landmarks(:, iSelected);
    bVisible = zeros(1, Mapa.nLandmarks);
    bVisible(1, iSelected) = 1; 
    
    % calculamos los pesos
    W = zeros(1, Robot.fpParticles);
    CovInv = inv(diag(Robot.senSigma).^2);
    for i = 1:Robot.fpParticles
        zPred = dist_angle(Robot.fpPos(:,i), landmark);
        % exp(-0.5*(zTrue-zPred)'*Robot.sensorCovInv*(zTrue-zPred))+0.01;
        W(i) = 1/((z-zPred)'*CovInv*(z-zPred)+0.001);
    end
    W = W/sum(W);


    % seleccionamos las mejores partículas
    CDF=cumsum(W)/sum(W);
    iSelect=rand(Robot.fpParticles,1);
    iNext=interp1(CDF,1:Robot.fpParticles,iSelect,'nearest','extrap');
    Best = Robot.fpPos(:,iNext);

    % estimamos según el método elegido

    if iMethod == 1 
        % pose de la mejor partícula
        [wMax, iMax] = max(W);
        xEst = Robot.fpPos(:, iMax); 

    elseif iMethod == 2
        % media de las poses
        xEst(1) = mean(Robot.fpPos(1, :));
        xEst(2) = mean(Robot.fpPos(2, :));        
        xEst(3) = mean(Robot.fpPos(3, :)+1000*pi)-1000*pi; % circ_mean

    elseif iMethod == 3
        xEst(1) = mean(Robot.fpPos(1, :).*W);
        xEst(2) = mean(Robot.fpPos(2, :).*W);        
        xESt(3) = mean(Robot.fpPos(3, :).*W+1000*pi)-1000*pi; % circ_mean
    end

end

