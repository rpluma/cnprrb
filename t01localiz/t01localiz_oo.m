%% Solución con orientación a objetos
% Sin ajustar la posición

clc; clear all; close all;
rng(5); % semilla para hacer el ejercicio reproducible
r=Robot(15, 10, [5;-5;pi/2]);
i = 1;
while i < 44
    if mod(i, 11) == 0
        r = r.Move([1, 0, pi/2]');
    else
        r = r.Move([1, 0, 0]');
    end
    r = r.Plot(i, i+1, true, true, true);
    i = i+1;
    pause(0.5);
end
r.PlotErrors();
%% Ajustando cada 5 pasos


rng(5); % semilla para hacer el ejercicio reproducible
r=Robot(15, 10, [5;-5;pi/2]);
i = 1;
while i < 44
    if mod(i, 11) == 0
        r = r.Move([1, 0, pi/2]');
    else
        r = r.Move([1, 0, 0]');
    end
    r = r.Plot(i, i+1, true, true, true);
    i = i+1;
    if mod(i, 6) == 0
        r = r.Fix();
        r = r.Plot(i, i+1, true, true, true);
        i = i + 1;
    end
    pause(0.5);
end
r.PlotErrors();