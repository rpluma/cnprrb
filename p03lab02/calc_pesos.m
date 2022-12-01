function [W] = calc_pesos(Robot, zTrue, landmark)
%CALC_PESOS Calcula los pesos de cada partícula
%   En lugar de la función propuesta se ha utilizado otra:
%   1) se calcula la diferencia entre la observación del sensor/partícula
%   2) se añade 0.001 para no dividir por cero
%   3) se invierte para que los más cercanos devuelvan valores más altos
    W = zeros(1, Robot.Particles);

    for i = 1:Robot.Particles
        zPred = dist_ang(Robot.xProb(:,i), landmark);
        % exp(-0.5*(zTrue-zPred)'*Robot.sensorCovInv*(zTrue-zPred))+0.01;
        W(i) = 1/((zTrue-zPred)'*Robot.sensorCovInv*(zTrue-zPred)+0.001);
    end
end

