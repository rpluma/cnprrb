function [z] = dist_angle(pose, landmark)
%DIST_ANGLE Devuelve la distancia y el ángulo desde la pose al landmark
% pruebas
%   dist_angle([0 0]', [sqrt(2) sqrt(2)]')
%   dist_angle([0 0]', [sqrt(2) -sqrt(2)]')
%   dist_angle([0 0]', [-sqrt(2) -sqrt(2)]')
%   dist_angle([0 0]', [-sqrt(2) sqrt(2)]')
    
    dist = norm(pose(1:2)-landmark(1:2)); % distancia euclídea
    angle = atan2(landmark(2)-pose(2),landmark(1)-pose(1)); % ángulo

    z = [dist angle_sum(angle, -pose(3))]'; % diferencia entre ángulos
end

