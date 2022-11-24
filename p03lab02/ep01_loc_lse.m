%% Ejercicio puntuable - LSE
% 
% 
%%
% * Alumno: Ricardo Panero Lamothe
% * Asignatura: Control y Programación de Robots
% * Curso: 2022/23
% * Plazo de entrega: 12-dic-2022
%
%% Enunciado
%
% Los sensores reales presentan ciertas limitaciones  físicas en cuanto al 
% rango y campo de visión. Modifica el ejercicio anterior para  contemplar 
% que el sensor sólo proporciona medidas en un rango limitado 𝑟𝑙 y una  
% orientación limitada ±𝛼 con respecto a la pose del robot. 
% 
% Podría darse el caso que no  existieran landmarks en el campo de visión 
% del sensor, con lo que el robot no dispondría de información sensorial 
% en una iteración
% 
% Aumenta el número de Landmarks a 10 y  considera el caso particular de un
% sensor con  un rango máximo de 20m de alcance, y un FOV  de ±60º
%
% Indica claramente en la memoria como tratas  el caso de no tener 
% suficientes observaciones, y muéstralo en las gráficas de posición y de 
% error. 
% 
% Añade/explica solo la parte del código  que has modificado
%
%% Planteamiento
%
% Aunque la práctica sólo pedía modificar el script, se ha refactorizado
% el código para agrupar variables relacionadas entre sí:
%
% * Mapa: incluye el número de landmarks, el tamaño y la lista de landmarks
% * Robot: posiciones verdadera, odométrica y estimada y resto de
% parámetros del robot
% * Sensor: rango, frente de visión (FOV) y resto de parámetros del sensor
% * Lse: parámetros del algoritmo LSE (iteraciones y tolerancia *)
%
% Adicionalmente, se han definido las siguientes funciones:
%
% * <https://github.com/rpluma/cnprrb/blob/main/p03lab02/crear_landmarks.m crear_landmarks>: 
% devuelve una lista de landmarks distribuidas aleatoriamente por el mapa.
% Inicializa las visualizaciones y dibuja el mapa
% * <https://github.com/rpluma/cnprrb/blob/main/p03lab02/leer_distancias.m leer_distancias>: 
% devuelve la lectura de distancia con ruido o -1 si el sensor no está en
% rango. Cambia el color de las landmarks a rojo/verde en función de si
% están en rango
%
% También se han modificado las siguientes funciones:
%
% * <https://github.com/rpluma/cnprrb/blob/main/p03lab02/AngleWrap.m AngleWrap>: 
% Se contempla la posibilidad de que el ángulo tenga más de 1 vuelta
% * <https://github.com/rpluma/cnprrb/blob/main/p03lab02/pose_comp.m pose_comp>: 
% Composición de dos poses utilizando la versión mejorada de AngleWrap
%


%% Implementación
%
% Realizamos el cuadrado siguiendo el siguiente algoritmo
%%
% # Inicializamos variables antes de entrar en el bucle
% # Para cada paso del bucle, decidimos la acción odométrica: avance/giro
% # Añadimos ruido a la acción odométrica para simular la acción real
% # Calculamos las nuevas posiciones odométrica y real mediante pose_comp
% # Leemos los sensores que están en rango
% # Si hay 3 o más sensores en rango, estimamos la posición con LSE
% # Si no, estimamos componiendo la acción odométrica desde la última estimación
% # Actualizamos errores y dibujamos el robot

% 1 Inicializamos variables antes de entrar en el bucle
clc;
clear all;
close all;
square_side = 10; % longitud del lado del cuadrado
length_update = 1; % cuántos metros se avanza en un paso
side_steps = square_side/length_update; % longitud/avance del paso
tot_steps = (side_steps+1)*4; % 4 lados + 4 giros
est_error = zeros(tot_steps,1); % error de localización estimada LSE
odo_error = zeros(tot_steps,1); % error de localización odométrica
num_sensors = zeros(tot_steps,1); % número de sensores en rango


% Mapa
Mapa={};
Mapa.nLandmarks = 10;
Mapa.Size = square_side*2;
Mapa.Landmarks = crear_landmarks(Mapa);

% Robot
Robot = {};
Robot.xTrue = [-square_side/2 -square_side/2 0]'; % abajo a la izquierda
Robot.xOdom = Robot.xTrue;
Robot.xEst = Robot.xTrue;
Robot.uAvanza = [length_update 0 0]'; % acción de avanzar
Robot.uGirizq = [0 0 pi/2]'; % acción de giro a la izquierda
Robot.uStd = [0.5 0.5 0.5*pi/180]'; % ruido de las acciones
Robot.uCov = diag(Robot.uStd) .^ 2; % covarianza de las acciones
plot(Robot.xTrue(1),Robot.xTrue(2),'kx');   % Real Position


% Sensor
Sensor = {};
Sensor.dStd = 0.05; % desviación típica del sensor de distancia
Sensor.dVar = Sensor.dStd ^2; % varianza del sensor de distancia
Sensor.rango = 20; % rango del sensor de distancia en metros
Sensor.alpha = 90; % rango angular (FOV) +/- grados

% Lse
Lse={};
Lse.max_iter = 100;
Lse.tolerance = 1.0e-09;  

for i=1:tot_steps
    % 2 Para cada paso del bucle, decidimos la acción odométrica: avance/giro
    if mod(i, side_steps+1)== 0
        Robot.uOdom = Robot.uGirizq;
    else
        Robot.uOdom = Robot.uAvanza;
    end

    % 3 Añadimos ruido a la acción odométrica para simular la acción real
    noise = Robot.uStd .* randn(3, 1);
    Robot.uTrue = pose_comp(Robot.uOdom, noise);

    % 4 Calculamos las nuevas posiciones odométrica y real mediante pose_comp
    Robot.xOdom = pose_comp(Robot.xOdom, Robot.uOdom);
    Robot.xTrue = pose_comp(Robot.xTrue, Robot.uTrue);

    % 5 Leemos los sensores que están en rango
    Sensor.z = leer_distancias(Mapa, Robot, Sensor);

    % 6 Si hay 3 o más sensores en rango, estimamos la posición con LSE
    if sum(Sensor.z>0) >= 3 
        Robot.xEst = estimar_posicion(Mapa, Robot, Sensor, Lse);
        color='go'; % dibujamos en verde porque se aplicó LSE
    else        
        % 7 Si no, estimamos componiendo la acción odométrica desde la última estimación
        Robot.xEst = pose_comp(Robot.xEst, Robot.uOdom);
        color='mo'; % dibujamos en naranja porque no aplicamos LSE
    end

    % 9 Actualizamos errores y dibujamos el robot
    odo_error(i) = norm(Robot.xTrue(1:2)-Robot.xOdom(1:2));
    est_error(i) = norm(Robot.xTrue(1:2)-Robot.xEst(1:2));
    num_sensors(i) = length(Sensor.z(Sensor.z>1));
    plot(Robot.xTrue(1),Robot.xTrue(2),'kx');   % Real Position
    plot(Robot.xOdom(1),Robot.xOdom(2),'r+');   % Odometry or ideal
    plot(Robot.xEst(1),Robot.xEst(2),color);  % the last estimation
    pause(0.1);
end

%% Resultados

figure(2); 
set(gcf,'Visible','on');

subplot(311); plot(odo_error,'r'); hold on; 
avg_odo_error = mean(odo_error);
line([1 size(odo_error,1)],[avg_odo_error avg_odo_error]); 
title('Odometric error'); ylabel('Error (m)');

subplot(312); 
plot(est_error,'g'); hold on; 
avg_est_error = mean(est_error);
line([1 size(est_error,1)],[avg_est_error avg_est_error]); 
title('Position Errors with LSE localization'); ylabel('Error (m)');

subplot(313); 
plot(num_sensors,'k'); hold on; 
line([1 size(num_sensors,1)],[3.5 3.5]); 
title('Sensors in range'); ylabel('# Sensors');
%xlabel('Iteration Number');
