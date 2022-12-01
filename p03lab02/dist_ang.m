function [z] = dist_ang(ps,ln)
%DIST_ANG Distancia y ángulo entre una pose y un landmark
%   La distancia euclídea se obtiene utilizando norm
%   Para el ángulo se usa atan2 y luego se normaliza
    dist = norm(ps(1:2)-ln(1:2));
    ang = AngleWrap(atan2(ln(2)-ps(2),ln(1)-ps(1)));
    z = [dist ang]';
end

