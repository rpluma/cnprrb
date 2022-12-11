%% Ejercicios Puntuables de Localización
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% Control y Programación de Robots
% 
% Ricardo Panero Lamothe
% 
% Diciembre de 2022
% 
% 
% 
% 
% 
% 
%% 
% 
% 
% 
% 
% 
% 
% 
%% Introducción
% Para el desarrollo de la tarea se ha reestructurado por completo el código 
% de los scripts proporcionados, intentando que la mayor parte sea reutilizable 
% en las dos prácticas.
% 
% En una primera iteración, se han desarrollado funciones para programar la 
% lógica de forma procedural, pero más tarde se ha reescrito el código utilizando 
% programación orientada a objetos.
% 
% Algunas funciones son comunes al enfoque de procedural y orientación a objetos:
%% 
% * <https://github.com/rpluma/cnprrb/blob/main/t01localiz/angle_sum.m angle_sum>: 
% Sustituye a la función AngleWrap. Suma dos ángulos y devuelve el resultado entre 
% -180º y 180º; para ello sumamos 10 vueltas y media, obtenemos el resto de dividir 
% por 360º y restamos la media vuelta de más
% * <https://github.com/rpluma/cnprrb/blob/main/t01localiz/dist_angle.m dist_angle>: 
% Calcula la distancia y ángulo desde una pose a una baliza y la devuelve en forma 
% de vector columna.
% * <https://github.com/rpluma/cnprrb/blob/main/t01localiz/comp_odom.m comp_odom>: 
% Composición odométrica de una pose con una actualización. Se define la matriz 
% de rotación y se usa álgebra matricial. El ángulo de la pose resultante se normaliza 
% a -180-180º por medio de la función angle_sum
% * <https://github.com/rpluma/cnprrb/blob/main/t01localiz/comp_noisy.m comp_noisy>: 
% Composición de una pose o con una actualización, pero añadiendo ruido al final. 
% El ruido simula que los actuadores del robot no son completamente precisos, 
% por lo que al realizar la acción el robot no llega al punto deseado. 
%% 
% Con el enfoque procedural se definieron las siguientes funciones:
%% 
% * <https://github.com/rpluma/cnprrb/blob/main/t01localiz/create_map.m create_map>: 
% Crea un mapa y genera un juego de balizas en posiciones aleatorias. Devuelve 
% una estructura que contiene el número de balizas, el tamaño del mapa y una matriz 
% de 2xn posiciones de balizas
% * <https://github.com/rpluma/cnprrb/blob/main/t01localiz/create_robot.m create_robot>: 
% Crea un robot e inicializa sus poses real, odométrica y estimada; registra los 
% parámetros de precisión de actuadores y sensores, así como el rango de los sensores; 
% inicializa los parámetros de configuración que requieren los algoritmos LSE 
% y FP; e inicializa las matrices para registrar errores y sensores que se utilizan 
% en cada iteración
% * <https://github.com/rpluma/cnprrb/blob/main/t01localiz/read_sensors.m read_sensors>: 
% Desde la posición real del robot, calcula la distancia y ángulo a cada una de 
% las balizas, y añade ruido para simular la imprecisión del sensor. Devuelve 
% tanto la lectura real como la lectura con ruido; también devuelve un vector 
% columna indicando si cada una de las balizas están visibles o no
% * <https://github.com/rpluma/cnprrb/blob/main/t01localiz/update_figure.m update_figure>: 
% Actualiza la figura que representa el mapa, la posición real del robot (x negra), 
% la posición odométrica (+ azul), la estimación LSE (círculo verde) o cuando 
% hay menos de 3 balizas a la vista (círculo rojo)
% * <https://github.com/rpluma/cnprrb/blob/main/t01localiz/est_lse.m est_lse>: 
% Estima la posición por mínimos cuadrados. El código es similar al que se proporcionó 
% en la práctica, salvo por que se utilizan las estructuras Robot y Mapa. La función 
% también recibe las lecturas del sensor (zNoisy) y qué balizas son visibles y 
% se pueden utilizar en el algoritmo (bVisible)
% * <https://github.com/rpluma/cnprrb/blob/main/t01localiz/est_fp.m est_fp>: 
% Estima la posición por filtro de partículas. Esta función recibe como entrada 
% la distancia y ángulo a todas las balizas (con ruido), aunque en cada invocación 
% sólo se selecciona una baliza
%% 
% El enfoque orientado a objetos se ha implementado reutilizando el código desarrollado 
% en el enfoque procedural. Hay una única clase <https://github.com/rpluma/cnprrb/blob/main/t01localiz/Robot.m 
% Robot> que contiene los siguientes métodos:
%% 
% * Constructor: Crea el robot y lo ubica en su posición inicial de un mapa 
% con un tamaño determinado y en el cual se ubican un determinado número de balizas 
% de forma aleatoria
% * Sense: Calcula la distancia y el ángulo entre la posición real del robot 
% y cada una de las balizas, añade ruido a las mediciones para simular la precisión 
% del sensor, y determina que´balizas son visibles según el rango del sensor y 
% cuál se utilizará al azar para el filtro de partículas
% * LSE: Estima la posición del robot por mínimos cuadrados si hay 3 o más balizas 
% visibles o compone la acción sobre la última estimación en caso contrario
% * FP: Estima la posición por filtro de partículas y selecciona las mejores 
% partículas para la siguiente iteración
% * Plot: Grafica las posiciones real y odométrica del robot desde la primera 
% iteración hasta el punto actual. Opcionalmente grafica las estimaciones LSE 
% y/o FP, así como la posición de las partículas en la última iteración
% * Move: Mueve todas las posiciones del robot (real, odométrica, posiciones 
% estimadas por LSE y por FP, y posiciones de las partículas) de acuerdo con la 
% petición uOdom; opcionalmente corrige la posición moviendo el robot hacia la 
% posición odométrica.
% * PlotErrors: Genera las distintas gráficas para representar los errores.
%% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
%% 1 - LSE
% Realizamos los ejercicios, incluyendo algunos que no son puntuables, para 
% demostrar la funcionalidad implementada.
% 1.0 Modelo de odometría
% _Diseña un programa que iterativamente comande un robot para  seguir una trayectoria 
% en forma de cuadrado de lado 10 metros_
% 
% _Modifica el ejercicio 2 para que la acción de control se vea afectada por 
% ruido. Para ello considera la matriz de covarianzas Σ𝑢𝑡 ._

clc; clear all; close all;
mapSize=15;numLmarks=10;tp=0.1;p0=[5;-5;pi/2];
seed=5; % 5, 11, 13, 18 18
rng(seed); 

figure;
rng(seed); 
r=Robot(mapSize, numLmarks, p0);
for i = 1:44
    if mod(i,11) == 0
        r=r.Move([0;0;pi/2], false);
    else
        r=r.Move([1;0;0], false);
    end
end
r.Plot(false, false, false);
%% 
% *Observaciones*
%% 
% * Según la pose odométrica, el robot describe un cuadrado perfecto. 
% * Sin embargo, la figura que describe el robot en la realidad dista mucho 
% de ser un cuadrado, y esto se debe a que los actuadores añaden ruido
% * El ruido en el ángulo tiene mucha influencia sobre el resultado final, ya 
% que a partir de ese punto los errores se propagan
% 1.1.a Implementación LSE
% _Analiza y completa el código que se adjunta con esta práctica (cypr_localization_LSE.mlx) 
% para que un robot, el cual es comandado a seguir un cuadrado de lado 10m, sea 
% capaz de estimar su posición real en cada iteración_
% 
% _Emplea  las medidas de distancia a 5 landmarks distribuidos en el entorno 
% de trabajo, y considera una matriz de covarianza del  sensor con valores bajos 
% (std_d=0.05 )_

figure;
rng(seed); 
r=Robot(mapSize, 5, p0); % 5 landmarks
r.senRange=[100;2*pi];   % sensor sin limmitación efectiva de distancia/ángulo
r.senSigma=[0.05;0.05];  % sensor más preciso (5 cm y 9º)
for i = 1:44
    if mod(i,11) == 0
        r=r.Move([0;0;pi/2], false);
    else
        r=r.Move([1;0;0], false);
    end
end
r.Plot(true, false, false);
%% 
% *Observaciones*
%% 
% * La figura real que describe el robot es muy distinta a la del ejercicio 
% anterior por el cambio en el número de balizas, ya que el generador de números 
% aleatorios ya no proporciona los mismos valores
% * Aunque la figura dista mucho de ser un cuadrado, la estimación de la posición 
% real es muy fiable (estrellas casi siempre dentro de los círculos)
% * En la configuración del robot (r.senRange), se ha establecido un rango de 
% 100m y un ángulo de 360º para que el sensor se comporte como si no tuviera limitaciones
% 1.1.b Implementación LSE
% _Vuelve a repetirlo con valores más altos (std_d=0.5)._

figure;
rng(seed); 
r=Robot(mapSize, 5, p0);    % 5 landmarks
r.senRange=[100;2*pi];      % sensor sin limmitación
r.senSigma=[0.5;.5];        % sensor poco preciso
for i = 1:44
    if mod(i,11) == 0
        r=r.Move([0;0;pi/2], false);
    else
        r=r.Move([1;0;0], false);
    end
end
r.Plot(true, false, false);
%% 
% *Observaciones*
%% 
% * La figura real que describe el robot es idéntica al caso anterior porque 
% seguimos utilizando la misma semilla.
% * Las estimaciones que realiza LSE son menos precisas, aunque aproximan bastante 
% bien la posición del robot
% 1.2. Ejercicio puntuable - LSE
% _Los sensores reales presentan ciertas limitaciones  físicas en cuanto al 
% rango y campo de visión. Modifica el ejercicio anterior para  contemplar que 
% el sensor sólo proporciona medidas en un rango limitado 𝑟𝑙 y una  orientación 
% limitada ±𝛼 con respecto a la pose del robot. Podría darse el caso que no  
% existieran landmarks en el campo de visión del sensor, con lo que el robot no 
% dispondría de información sensorial en una iteración._
% 
% _Aumenta el número de Landmarks a 10 y  considera el caso particular de un 
% sensor con  un rango máximo de 20m de alcance, y un FOV  de ±60º_
% LSE con 3 o más balizas visibles
% Representamos los 18 primeros movimientos del robot en los que hay al menos 
% 3 balizas visibles desde la posición real del robot

figure;
rng(seed); 
r=Robot(mapSize, 10, p0);   % 10 landmarks
r.senRange=[20;60*pi/180];  % rango de 20 metros y 60º
r.senSigma=[0.05;.05];      % sensor muy preciso
for i = 1:18
    if mod(i,11) == 0
        r=r.Move([0;0;pi/2], false);
    else
        r=r.Move([1;0;0], false);
    end
end
r.Plot(true, false, false);
%% 
% *Observaciones*
%% 
% * Anque el enunciado no lo especifica, hemos optado por versión más fiable 
% del sensor
% * Aunque las balizas están siempre en las mismas posiciones, sólo se etiquetan 
% las que están en el rango de visión del robot desde la posición real.
% * En todo el trazado hasta el punto de la figura, la estimación se realiza 
% con 3 o más balizas a la vista, pero la baliza L9 deja de estar visible para 
% la siguiente estimación, que se tiene que realizar por otro método a partir 
% de ese punto.
% Tratamiento con menos de 3 balizas
% _Indica claramente en la memoria como tratas  el caso de no tener suficientes 
% observaciones,  y muéstralo en las gráficas de posición y de  error. Añade/explica 
% solo la parte del código  que has modificado._ 
% 
% Como con dos balizas no podemos utilizar LSE, partiendo de la última posición 
% estimada, componemos la actualización odométrica sobre la última estimación

r=r.Move([1;0;0], false);
r.Plot(true, false, false);
a = annotation('arrow'); a.Parent = gca; a.Position = [-4.2, 5,-1,0];
a = annotation('arrow'); a.Parent = gca; a.Position = [-4.2,-2.8,-1,0];
%% 
% *Observaciones*
%% 
% * En este caso, el robot se está desplazando hacia abajo pero "piensa" que 
% se está moviendo hacia la derecha.
% * La nueva posición se estima moviendo hacia la derecha la estimación anterior, 
% pero esto aleja la estimación y la posición real, ya que los ángulos de la pose 
% odométrica y de la pose real son muy distintos.
% * En la medida en que los ángulos difieran, la estimación por este método 
% será más o menos fiable.
% Resultado final con LSE
% Completamos el cuadrado para evaluar el resultado final

for i = 20:44
    if mod(i,11) == 0
        r=r.Move([0;0;pi/2], false);
    else
        r=r.Move([1;0;0], false);
    end
end
r.Plot(true, false, false);
%% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% *Observaciones*
%% 
% * Las estimaciones son fiables cuando hay 3 o más balizas visibles, lo cual 
% ocurre en la mayor parte del recorrido.
% * Cada vez que el robot deja de tener 3 balizas a la vista y que el ángulo 
% de la pose real difiere significativamente del de la pose odométrica, la estimación 
% "se pierde" porque mueve el robot en la dirección equivocada
% * La primera vez que ocurre, la estimación "camina" hacia la izquierda porque 
% el robot debería estar en la parte superior del cuadrado, aunque el desplazamiento 
% real es hacia abajo
% * Del mismo modo, cuando se vuelve a perder en el lado inferior del cuadrado 
% el robot sigue caminando hacia abajo porque en ese punto debería estar bajando 
% por el lado izquierdo del cuadrado
% * Cuando el robot recupera la visión de 3 o más balizas, la estimación se 
% corrige por completo
%% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
%% 2 Filtro de Partículas
% 2.1 Implementación del filtro de partículas
% _En este ejercicio vamos a implementar paso a paso un sistema de  localización 
% basado en filtro de partículas._
% 
% _En este ejercicio vamos a implementar paso a paso un sistema de  localización 
% basado en filtro de partículas. Para ello se recomienda seguir los  siguientes 
% pasos:_ 
% 
% _*a. Medida Sensorial:* Utilizando un mapa de landmarks como el empleado en 
% los  ejercicios anteriores, implementa ahora una función que tome como entrada 
% la  pose del robot, seleccione aleatoriamente un solo landmark de entre todos 
% los  disponibles, y devuelva la observación del sensor a dicho landmark. Dicha  
% observación estará compuesta por las medidas de distancia y ángulo entre el 
% robot  y el landmark seleccionado, afectadas por el ruido del sensor siguiendo 
% su modelo  de ruido en base a la matriz de covarianza_
% 
% El método Sense implementa esta funcionalidad:
%%
% 
%   for i = 1:r.numLmarks
%     % cálculo de distancia y ángulo
%     r.zTrue(:, i) = dist_angle(r.pTrue, r.posLmarks(:, i));
%     % añadir ruido a las lecturas del sensor
%     r.zNoisy(:, i) = r.zTrue(:,i) + r.senSigma .*randn(2, 1);
%     % selección de sensores visibles (utilizado en LSE)
%     r.bVisible(i) = r.zTrue(1,i)<=r.senRange(1) && abs(r.zTrue(2,i)) <= r.senRange(2));        
%   end
%   % número de sensores visibles (utilizado en LSE)
%   r.nVisible = sum(r.bVisible);
%   % baliza elegida en cada paso
%   r.iLmFP = randperm(r.numLmarks, 1); 
%
%% 
% 
% 
% _*b. Acción de Control:* Programa el movimiento del robot como en prácticas  
% anteriores, buscando realizar un cuadrado de lado 10m, y estando las acciones 
% de  control afectadas por cierto ruido aleatorio en base a_ $\Sigma_s$
% 
% _En este punto, tendremos la pose odométrica (xOdom), es decir, donde el robot  
% cree que está, la pose real (xTrue), es decir, donde el robot está realmente 
% (esta  pose, aunque la simularemos en este ejercicio, es desconocida por el 
% robot y es la  que se quiere estimar con el sistema de localización), y deberemos 
% generar un  número de N poses candidatas (samples), dadas por las partículas 
% del filtro que  estamos implementando. Considerar N=100_
% 
% Esta funcionalidad se ha implementado dentro del método Move:
%%
% 
%   % acción de control del robot afectado por el ruido aleatorio del actuador
%   r.pTrue = comp_noisy(r.pTrue, uOdom, r.actSigma);
%   % la posición odométrica sigue el cuadrado perfecto (comp_odom)
%   r.pOdom = comp_odom(r.pOdom, uOdom);
%   % las partículas también se muevoen pero también con ruido (comp_noisy)
%   r.pPart = comp_noisy(r.pPart, uOdom, r.actSigma);
%
%% 
% 
% 
% _*c. Actualización de Poses:* Programa que en cada iteración el robot realiza 
% una acción  de control (para avanzar en el cuadrado) y recibe una medición del 
% sensor a un  landmark aleatorio del mapa (usando la función del apartado anterior). 
% Con esta  información, deberemos actualizar las poses xOdom y xTrue, así como 
% las poses de  todas las samples (considerando para cada una de ellas una muestra 
% de ruido  diferente). Visualiza en cada iteración estas poses_
% 
% Esta funcionalidad está implementada en el método Move (ver apartado anterior). 
% Adicionalmente se ha implementado un método para visualizar el robot desde la 
% posición inicial hasta el último paso.
% 
% _*d. Pesos del FP*: Para poder estimar la pose real del robot a partir de 
% las N partículas  del filtro, partimos de que la posición de los landmarks es 
% conocida en el mapa. El  objetivo ahora es poder discernir cuales de las N partículas 
% actuales representan  adecuadamente la pose real del robot y cuáles no (recuerda 
% que la pose xTrue es  desconocida, y por tanto no se puede usar en los cálculos). 
% Para ello, asocia a cada  partícula un peso W(i) que indique cómo de probable 
% es la pose que representa  dicha partícula con respecto a las observaciones 
% que toma el robot. Inicialmente  consideraremos que todas las partículas tienen 
% el mismo peso, y por comodidad  impondremos que la suma de todos los pesos resulte 
% 1 en todo momento  (normalización)_
% 
% Esta funcionalidad se ha implementado dentro del método FP. En lugar de la 
% expresión sugerida en el enunciado (comentada abajo) se ha optado por una función 
% que no devuelve rangos tan extremos
%%
% 
%   % utilizamos la baliza elegida al azar (slm=Selected LandMark)
%   slm = r.posLmarks(:, r.iLmFP); 
%   
%   % utilizamos la medida de distancia y ángulo con ruido (zNoisy)
%   z = r.zNoisy(: , r.iLmFP); 	
%   
%   % inicialización de pesos
%   W = zeros(1, r.fpPart);
%   
%   % inversa de matriz de covarianza del sensor
%   CovInv = inv(diag(r.senSigma).^2);
%   
%   for i = 1:r.fpPart
%       % predicción de distancia y ángulo desde la partícula a la baliza
%       zPred = dist_angle(r.pPart(:,i), slm);
%      
%       % Peso = 1/error entre predicción y de partícula y medida observada    
%       W(i) = 1/((z-zPred)'*CovInv*(z-zPred)+0.001);
%       % W(i)=exp(-0.5*(zTrue-zPred)'*r.sensorCovInv*(zTrue-zPred))+0.01;
%   end
%   % normalización de pesos
%   W = W/sum(W);
%
%% 
% _*e. Selección de Partículas:* Hasta ahora mantenemos el mismo grupo de N 
% partículas,  actualizando su pose en cada iteración y recalculando sus pesos. 
% No obstante,  seguimos manteniendo de una iteración a otra todas las partículas 
% sin importar sus  pesos, con lo que observamos como las partículas se van dispersando 
% y alejando de  la posición real del robot (al igual que ocurría en la práctica 
% 1, por efecto del ruido  en la acción de control)._
% 
% La selección se realiza con el método propuesto
% 
% _*f. Estimación de la pose del robot (xEst):* Una vez hemos seleccionado las 
% partículas  con mejor peso, ya tenemos implementado el filtro de partículas. 
% Tan solo nos falta  determinar, en base a las partículas actuales y sus pesos,_
% 
% En el método Move se han implementado los 3 métodos sugiredos, aunque sólo 
% se está utilizando el últmo porque se ha observado que es el más robusto.
%%
% 
%   % pose de la mejor partícula
%   if r.fpMethod == "best" 
%     [wMax, iMax] = max(W);
%     r.pEstF = r.pPart(:, iMax); 
%   
%   % media de poses de partículas
%   elseif r.fpMethod == "mean" 
%    r.pEstF(1) = sum(r.pPart(1, :));
%    r.pEstF(2) = sum(r.pPart(2, :));        
%    r.pEstF(3) = circ_mean(r.pPart(3, :)');
%   
%   % media ponderada de poses
%   elseif r.fpMethod == "weight" 
%    r.pEstF(1) = sum(r.pPart(1, :).*W);
%    r.pEstF(2) = sum(r.pPart(2, :).*W);
%    r.pEstF(3) = circ_mean(r.pPart(3, :)', W');
%   end	
%
% 2.2.a Filtro sin correcciones
% _Una vez tengas implementado y funcionando  correctamente el filtro de partículas 
% del ejercicio anterior, vamos a corregir la pose del  robot. Sabemos que el 
% robot se “desvía” de su trayectoria deseada (el cuadrado de  lado 10m), pero 
% gracias a la localización del FP ya sabemos dónde estamos realmente.  Implementa 
% en este ejercicio, que cada 5 iteraciones se añade un “paso extra” para  corregir 
% la pose del robot y hacer que vuelva al cuadrado._
% _Funcionamiento del filtro_
% _Muestra inicialmente una captura en la que se aprecie el correcto  funcionamiento 
% del filtro (sin correcciones)._ 

figure;
rng(seed); 
r=Robot(mapSize, 10, p0);   % 10 landmarks
r.senRange=[20;60*pi/180];  % rango de 20 metros y 60º
r.senSigma=[0.05;.05];      % sensor muy preciso
for i = 1:44
    if mod(i,11) == 0
        r=r.Move([0;0;pi/2], false);
    else
        r=r.Move([1;0;0], false);
    end
end
r.Plot(false, true, true);
%% 
% *Observaciones*
%% 
% * Sólo se muestran las partículas del último paso, pero para ver la evolición 
% del movimento de las partículas se puede invocar al método de visualización 
% en cada paso.
% * La posición real del robot es idéntica a la del ejercicio anterior porque 
% en cada paso del robot se están calculando ambas estimaciones y solo al final 
% se decide cuál de ellas mostrar.
% * Con una sola baliza elegida al azar, el filtro de partículas estima bastante 
% bien la posición del robot, y no tiene el problema de "desorientación" que observábamos 
% con LSE, ya que ahora disponemos de una estimación del ángulo de la pose que 
% con LSE no teníamos.
% Error en la posición
% _Muestra una gráfica del error  entre xTrue y xEst a lo largo de las iteraciones, 
% tanto error en posición como en  orientación. Comenta los resultados._

r.PlotErrors(true, false, false, false);
%% 
% *Observaciones*
% 
% La figura muestra la comparativa de los 3 errores (odométrico, LSE y FP) que 
% se han calculado a partir de los mismos datos
%% 
% * El error odométrico está midiendo la distancia entre la posición que queríamos 
% tener y la posición real del robot. Este error depende de la fiabilidad de los 
% actuadores, sobre todo en los giros. A partir del tercer paso el robot empieza 
% a desviarse hacia la izquierda y este error se va propagando hasta el final. 
% La media del error depende mucho de la trayectoria seguida (imprevisible), y 
% en este caso ha resultado ser de 5 metros.
% * El error de la estimación con LSE está calculando la distancia entre la 
% estimación realizada y la posición real. La media del error es muy media cercana 
% a cero cuando hay 3 o más balizas en el rango de visión, pero se dispara cada 
% vez que el robot "se pierde" (en los pasos 19, 25 y 41) llegando a estar por 
% encima de 6 metros. Aunque el error es bajo, este método no permite hacer correcciones 
% porque no hemos estimado el ángulo de la pose.
% * El error con la estimación del FP es análogo al medido con LSE. Aunque el 
% error es mayor que el obtenido con LSE (medio metro de promedio), hay que tener 
% en cuenta que sólo se ha utilizado una baliza, y que además nos permite hacer 
% correcciones porque FP proporciona una estimación del ángulo de la pose.
% 2.2.b Filtro con correcciones
% Captura de correcciones cada 5 pasos
% _Luego añade otra captura donde se aprecien las correcciones cada 5 pasos  
% implementados en este ejercicio._ 
% 
% Para implementar las correcciones añadimos el parámetro variable bFix al método 
% Move.

figure; set(gcf,'Visible','on');
rng(seed); 
r=Robot(mapSize, 10, p0); % 5 landmarks
r.senRange=[20;60*pi/180]; % rango de 20 metros y 60º
r.senSigma=[0.05;.05]; % sensor poco preciso
for i = 1:44
    bFix = mod(i,5)==0;
    if mod(i,11) == 0
        r=r.Move([0;0;pi/2], bFix);
    else
        r=r.Move([1;0;0], bFix);
    end
    r.Plot(false, true, true);
    pause(0.5)
end
%% 
% *Observaciones*
%% 
% * Aunque el robot dibuja una figura muy distinta a un cuadrado, el resultado 
% se ajusta mejor que cuando no había correcciones.
% * La corrección más visible se produce llegando a la esquina superior derecha 
% del cuadrado, y se aprecia que el robot vuelve a la posición odométrica con 
% bastante exactitud
% Error de posición y orientación
% En este caso, muestra las gráficas de error  entre xEst y xOdom (para que 
% se aprecie que cada 5 pasos el error debe  hacerse muy pequeño), tanto de posición 
% como de orientación. 

r.PlotErrors(false, false, true, false); 
%% 
% *Observaciones*
%% 
% * El error odométrico se ha reducido a menos de un metro en promedio; esto 
% se debe a que las correcciones redirigen el robot a la posición requerida.
% * El error en la posición estimada por el filtro es del mismo orden de magnitud 
% que cuando no se hacían correciones.
% * El error en el ángulo se ha calculado como diferencia entre la pose real 
% del robot y la pose estimada. Hay oscilaciones significativas (p. ej. cerca 
% de +40º en el paso 19 y cerca de -30º en el paso 4), pero en general los ángulos 
% coinciden bastante bien.
%% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
%% Apéndice - Ejecución manual
% Para probar el buen funcionamiento del robot, hemos definido un script que 
% permite ejecutar movimientos simples eligiendo en cada caso qué queremos que 
% haga el robot
%% 
% * Inicialización: empezamos desde cero utilizando siempre la misma semilla 
% para que las pruebas sean reproducibles
% * Avanzar: el robot avanza un metro sin corregir su pose
% * Corregir sin avanzar: la posición odométrica se mantiene pero el robot se 
% desplaza hacia ella desde su estimación FP y ajusta su ángulo para que siga 
% le misma orientación
% * Giro a la izquierda: el robot gira 90 grados a la izquierda y después avanza 
% una posición permitiendo así ver que efectivamente ha girado como deseamos
% * Giro a la derecha: análoga al giro a la izquierda
%% 
% Las figuras se van actualizando sobre la anterior, por lo que cada una representa 
% el estado del robot la última vez que se ejecutó el script
% Inicialización

clc; clear all; close all;
mapSize=15;numLmarks=10;tp=0.1;p0=[5;-5;pi/2];
seed=5; % 5, 11, 13, 18 18
rng(seed); 
figure
r=Robot(mapSize, numLmarks, p0);
r.Plot(true, true, true);
% Avanzar

r=r.Move([1;0;0], false);
r.Plot(true, true, true);
% Corregir sin avanzar

r=r.Move([0;0;0], true);
r.Plot(true, true, true);
% Giro a la izquierda

r=r.Move([0;0;pi/2], false);
r=r.Move([1;0;0], false);
r.Plot(true, true, true);
% Giro a la derecha

r=r.Move([0;0;-pi/2], false);
r=r.Move([1;0;0], false);
r.Plot(true, true, true);