%% Ejercicio puntuable - FP
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
% Una vez tengas implementado y funcionando correctamente el filtro de partículas
% del ejercicio anterior, vamos a corregir la pose del robot. 
%
% Sabemos que el robot se “desvía” de su trayectoria deseada (el cuadrado de 
% lado 10m), pero gracias a la localización del FP ya sabemos dónde estamos realmente. 
% Implementa en este ejercicio, que cada 5 iteraciones se añade un “paso extra” para 
% corregir la pose del robot y hacer que vuelva al cuadrado.
%
%% a. Filtro sin correcciones
%
% Muestra inicialmente una captura en la que se aprecie el correcto 
% funcionamiento del filtro (sin correcciones). Muestra una gráfica del error 
% entre xTrue y xEst a lo largo de las iteraciones, tanto error en posición como en 
% orientación. Comenta los resultados.
%
%% b Correcciones cada 5 pasos
%
% Luego añade otra captura donde se aprecien las correcciones cada 5 pasos 
% implementados en este ejercicio. En este caso, muestra las gráficas de error 
% entre xEst y xOdom (para que se aprecie que cada 5 pasos el error debe 
% hacerse muy pequeño), tanto de posición como de orientación. 
%
% MUY IMPORTANTE: Recuerda que no podemos usar xTrue en los cálculos de 
% esta corrección, tan solo xEst.
%
%% Planteamiento
%
% Para la implementación de la funcionalidad se han definido las siguientes funciones:
% <https://github.com/rpluma/cnprrb/blob/main/p03lab02/leer_sensor.m leer_sensor>: 
% elige aleatoriamente un sensor del mapa y devuelve una observación afectada de ruido
% con la distancia y ángulo entre el robot y el sensor
% <https://github.com/rpluma/cnprrb/blob/main/p03lab02/mover_particulas.m mover_particulas>: 
% para cada partícula, genera ruido aleatorio y mueve la partícula a la nueva posición
% <https://github.com/rpluma/cnprrb/blob/main/p03lab02/calc_pesos.m calc_pesos>: 
% calcula el peso de cada partícula en función de si la estimación de su posición con 
% respecto al landmark se asemeja a la medición desde la posición real

%% a) Medida sensoríal
%
% Utilizando un mapa de landmarks como el empleado en los  ejercicios anteriores, 
% implementa ahora una función que tome como entrada la  pose del robot, seleccione 
% aleatoriamente un solo landmark de entre todos los  disponibles, y devuelva 
% la observación del sensor a dicho landmark. Dicha  observación estará compuesta 
% por las medidas de distancia y ángulo entre el robot  y el landmark seleccionado, 
% afectadas por el ruido del sensor siguiendo su modelo  de ruido en base a la 
% matriz de covarianza

clc;
clear all;
close all;
square_side = 10; % longitud del lado del cuadrado
length_update = 1; % cuántos metros se avanza en un paso
side_steps = square_side/length_update; % longitud/avance del paso
tot_steps = (side_steps+1)*4; % 4 lados + 4 giros
est_error = zeros(tot_steps,1); % error de localización estimada LSE
odo_error = zeros(tot_steps,1); % error de localización odométrica

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
%Robot.sensorStd = [0.5 0.5*pi/180]; % desviación típica del sensor
Robot.sensorStd = [0.11 0.5*pi/180]; % 1 cm de desviación y medio grado
%Robot.sensorCov = diag(Robot.sensorStd.^2); % matriz covarianza
Robot.sensorCov = diag(Robot.sensorStd); % matriz covarianza
Robot.sensorCovInv = inv(Robot.sensorCov);
plot(Robot.xTrue(1),Robot.xTrue(2),'kx');   % Real Position

% Sensor
[zTrue,landmark]=leer_sensor(Mapa, Robot);

%% b) Acción de control
%
% Programa el movimiento del robot como en prácticas  anteriores, buscando realizar 
% un cuadrado de lado 10m, y estando las acciones de  control afectadas por cierto 
% ruido aleatorio en base a la matriz de covarianza de acciones
% 
% En este punto, tendremos la pose odométrica (xOdom), es decir, donde el robot  
% cree que está, la pose real (xTrue), es decir, donde el robot está realmente 
% (esta  pose, aunque la simularemos en este ejercicio, es desconocida por el 
% robot y es la  que se quiere estimar con el sistema de localización), y deberemos 
% generar un  número de N poses candidatas (samples), dadas por las partículas 
% del filtro que  estamos implementando. Considerar N=100.

Robot.uAvanza = [length_update 0 0]'; % acción de avanzar
Robot.uGirizq = [0 0 pi/2]'; % acción de giro a la izquierda
Robot.uStd = [0.01 0.01 0.5*pi/180]'; % ruido de las acciones
Robot.uCov = diag(Robot.uStd) .^ 2; % covarianza de las acciones
Robot.Particles = 100;
Robot.xProb = zeros(3, Robot.Particles);

%% c) Actualización de poses
%
% Programa que en cada iteración el robot realiza una acción  de control (para 
% avanzar en el cuadrado) y recibe una medición del sensor a un  landmark aleatorio 
% del mapa (usando la función del apartado anterior). Con esta  información, deberemos 
% actualizar las poses xOdom y xTrue, así como las poses de  todas las samples 
% (considerando para cada una de ellas una muestra de ruido  diferente). Visualiza 
% en cada iteración estas poses.

Robot.uOdom = Robot.uAvanza;
Robot.xProb = mover_particulas(Robot, true);
Robot.xProb = mover_particulas(Robot, false);
Robot.xProb = mover_particulas(Robot, false);
Robot.xProb = mover_particulas(Robot, false);
Robot.xProb = mover_particulas(Robot, true);

%% d) Pesos del FP
%
% Para poder estimar la pose real del robot a partir de las N partículas  del 
% filtro, partimos de que la posición de los landmarks es conocida en el mapa. 
%
% El  objetivo ahora es poder discernir cuales de las N partículas actuales representan  
% adecuadamente la pose real del robot y cuáles no (recuerda que la pose xTrue 
% es  desconocida, y por tanto no se puede usar en los cálculos). Para ello, asocia 
% a cada  partícula un peso W(i) que indique cómo de probable es la pose que representa  
% dicha partícula con respecto a las observaciones que toma el robot. Inicialmente  
% consideraremos que todas las partículas tienen el mismo peso, y por comodidad  
% impondremos que la suma de todos los pesos resulte 1 en todo momento  (normalización).
% 
% El peso de cada partícula W(i) se actualiza en cada iteración atendiendo a 
% la  diferencia existente entre la medida del sensor (distancia y ángulo afectados 
% por  ruido) que se ha obtenido en este paso (ver apartado a), y la distancia 
% y ángulo que  hay desde la partícula considerada al landmark seleccionado. Es 
% decir: 
% 
% 𝑊(𝑖) = 𝑓(z - zPre)
% 
% donde z es la medida sensorial que toma el robot desde el ground-truth con 
% el  sensor real (𝑟, 𝜙) y por tanto afectado por ruido, y zPred es la medida 
% hipotética  que tomaría desde cada una de las partículas hasta el mismo landmark. 
% La función f puede ser cualquier función que asigne mayores valores a aquellas 
% partículas con la menor discrepancia, es decir, con z-Zpred más bajos. Una posible 
% función es: 
% 
% W(i)=exp(-0.5*(z-zPred)'*inv(Sigma_sensor)*(z-zPred))+0.001; 
% 
% Es decir, una función gaussiana utilizando la matriz de covarianza del modelo 
% del  sensor. Implementa esta función para que se vayan actualizando los pesos 
% de las  partículas en cada iteración.

W = calc_pesos(Robot, zTrue, landmark);

%% e) Selección de partículas
%
% Hasta ahora mantenemos el mismo grupo de N partículas, actualizando su pose en 
% cada iteración y recalculando sus pesos. 
%
% No obstante, seguimos manteniendo de una iteración a otra todas las partículas
% sin importar sus pesos, con lo que observamos como las partículas se van dispersando
% y alejando de la posición real del robot (al igual que ocurría en la práctica 1,
% por efecto del ruido en la acción de control). 
%
% El objetivo en este apartado es conseguir que el grupo de N partículas esté siempre 
% en torno a la pose real (con cierta variación). Para conseguir esto debemos 
% seleccionar las partículas, buscando quedarnos con aquellas que tengan mayor 
% peso, y descartando las de pesos menores. Para ello vamos a seleccionar 
% aleatoriamente en base a sus pesos aquellas partículas que queremos mantener, 
% de forma que las que tienen un peso bajo tendrán poca probabilidad de sobrevivir
% de una iteración a la siguiente

%% f) Estimación de la pose del robot
%
% Existen muchas formas de hacer esta selección. Nosotros la implementaremos 
% siguiendo el siguiente código:

% Compute the cumulative densitity function as: 
CDF=cumsum(W)/sum(W);

% Select random numbers from the standard uniform distribution [0,1)
iSelect=rand(Robot.Particles,1);

% Interpolate the values randomly selected: options nearest 
% and extrap are needed to avoid values out of range. iNext are 
% the indexes of the chosen particles
iNext=interp1(CDF,1:Robot.Particles,iSelect,'nearest','extrap');

% Update the samples set. Some particles can be repetitions 
Robot.xProb = Robot.xProb(:,iNext);


%% f. Estimación de la pose del robot (xEst)

% Una vez hemos seleccionado las partículas con mejor peso, ya tenemos 
% implementado el filtro de partículas. Tan solo nos falta determinar, 
% en base a las partículas actuales y sus pesos, ¿cuál es la posición 
% estimada del robot? Existen varias opciones posibles:
%
% i. Seleccionar como xEst aquella partícula que tenga el máximo peso (la 
% pose más prometedora según la última medida sensorial).
[wMax, iMax] = max(W);
Robot.xEst = Robot.xProb(:, iMax)


% ii. Otra opción es calcular la media entre todas las partículas.
Robot.xEst(1) = mean(Robot.xProb(1, :));
Robot.xEst(2) = mean(Robot.xProb(2, :));
% Robot.xEst(3) = circ_mean(Robot.xProb(3, :));
Robot.xESt(3) = mean(Robot.xProb(3, :)+1000*pi)-1000*pi;

% iii. Una opción más “inteligente” es calcular la media de todas las 
% partículas, pero ponderada en función del peso de cada una.
%Robot.xEst(1) = sum(Robot.xProb(1, :)*W)/(Robot.Particles*sum(W));
%Robot.xEst(2) = sum(Robot.xProb(2, :)*W)/(Robot.Particles*sum(W));
%Robot.xESt(3) = sum((Robot.xProb(1, :)+1000*pi)*W)/(Robot.Particles*sum(W))-1000*pi;


%
% Nota: Para el cálculo de xEst como media de las partículas, hay que tener cuidado
% con la rotación del robot, puesto que al ser un valor entre [-pi, pi] no podemos usar 
% una media aritmética, sino una media circular. Para ello se recomienda usar la 
% función “circ_mean” de la Circular Statistical Toolbox





%% Implementación

%% a. Filtro sin correcciones
%
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

    % lectura de un sensor aleatorio (ver apartado c)
    [zTrue,landmark]=leer_sensor(Mapa, Robot);

    % generación de partículas y visualizar cada 5 pasos
    Robot.xProb = mover_particulas(Robot, mod(i, 5)==0);

    % calcular pesos
    W = calc_pesos(Robot, zTrue, landmark);

    % seleccionar las mejores partículas
    CDF=cumsum(W)/sum(W);
    iSelect=rand(Robot.Particles,1);
    iNext=interp1(CDF,1:Robot.Particles,iSelect,'nearest','extrap');
    Robot.xProb = Robot.xProb(:,iNext);

    % actualizar posición
    [wMax, iMax] = max(W);
    Robot.xEst = Robot.xProb(:, iMax)
    
end

%% b Correcciones cada 5 pasos

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
%Robot.sensorStd = [0.5 0.5*pi/180]; % desviación típica del sensor
Robot.sensorStd = [0.11 0.5*pi/180]; % 1 cm de desviación y medio grado
%Robot.sensorCov = diag(Robot.sensorStd.^2); % matriz covarianza
Robot.sensorCov = diag(Robot.sensorStd); % matriz covarianza
Robot.sensorCovInv = inv(Robot.sensorCov);
plot(Robot.xTrue(1),Robot.xTrue(2),'kx');   % Real Position
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

    % lectura de un sensor aleatorio (ver apartado c)
    [zTrue,landmark]=leer_sensor(Mapa, Robot);

    % generación de partículas y visualizar cada 5 pasos
    Robot.xProb = mover_particulas(Robot, true);

    % calcular pesos
    W = calc_pesos(Robot, zTrue, landmark);

    % seleccionar las mejores partículas
    CDF=cumsum(W)/sum(W);
    iSelect=rand(Robot.Particles,1);
    iNext=interp1(CDF,1:Robot.Particles,iSelect,'nearest','extrap');
    Robot.xProb = Robot.xProb(:,iNext);

    % actualizar posición
    [wMax, iMax] = max(W);
    Robot.xEst = Robot.xProb(:, iMax)

    if (mod(i, 5)==0)
        xOdom = xEst;
    end
    
end

%% TODO REVISAR----------------------------------------------------------------------------------------
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


