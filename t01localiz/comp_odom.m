function [cDest] = comp_odom(cOrig, cUpdate)
%COMP_ODOM Composición odométrica de poses
%   cOrig es un vector columna con la pose del robot en el origen
%   cUpdate es un vector columna con la actualización a realizar
%   cDest es un vector columna con la poses teórica alcanzada
%
%   ejemplos
%   comp_odom([0, 0, -pi/4]', [sqrt(2) 0 0]') % [1, -1, -pi/4]'
%   comp_odom([0, 0, 0]', [1,0,0; 0,1,0; 0,0,7*pi/4]')
    
    % Matriz de rotación
    R = [ cos(cOrig(3)) sin(cOrig(3)) 0;
         -sin(cOrig(3)) cos(cOrig(3)) 0;
                      0             0 1];
    % Pose de destino 
    cDest = cOrig + (cUpdate' * R)';

    % Normalización de ángulos a -pi,pi (sumando 0 al ángulo)
    cDest(3) = angle_sum(cDest(3), 0);
end

