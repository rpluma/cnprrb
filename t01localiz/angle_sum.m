function [thout] = angle_sum(th1,th2)
%ANGLE_SUM Suma de dos ángulos en radianes
%   Devuelve el resultado de sumar los ángulos en el rango (-pi,pi)
%   th1 y th2 pueden ser escalares o vectores
%   para asegurar el rango de salida se añaden n vueltas y media a th2
%   con la función mod se eliminan las vueltas adicionales
%   y para ajustar el rango de salida se resta la media vuelta adicional
%  
%   ejemplos:
%       angle_sum(pi, pi/4)*180/pi % 180º+45º=-135º
%       angle_sum(-2*pi:30*pi/180:2*pi, 0)*180/pi % -360º a +360º
    thout = mod(th1 + (th2+11*pi), 2*pi)-pi;
end

