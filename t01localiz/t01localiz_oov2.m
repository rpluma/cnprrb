%% Implementación con orientación a objetos
%% 
%% Ejecución manual
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
% Inicializacion

clc; clear all; close all;
mapSize=15;numLmarks=10;tp=0.1;p0=[5;-5;pi/2];
seed=5; % 5, 11, 13, 18 18
rng(seed); 
figure
r=Robot(mapSize, numLmarks, p0);
r.NewPlot(true, true, true);
% Avanzar

r=r.Move([1;0;0], false);
r.NewPlot(true, true, true);
% Corregir sin avanzar

r=r.Move([0;0;0], true);
r.NewPlot(true, true, true);
% Giro a la izquierda

r=r.Move([0;0;pi/2], false);
r=r.Move([1;0;0], false);
r.NewPlot(true, true, true);
% Giro a la derecha

r=r.Move([0;0;-pi/2], false);
r=r.Move([1;0;0], false);
r.NewPlot(true, true, true);
%% Ejecución programada
% Una vez que tenemos el robot funcionando, realizamos todos los ejercicios 
% incluyendo los que no son puntuables
% 0 Modelo de odometría
% _Diseña un programa que iterativamente comande un robot para  seguir una trayectoria 
% en forma de cuadrado de lado 10 metros_
% 
% _Modifica el ejercicio 2 para que la acción de control se vea afectada por 
% ruido. Para ello considera la matriz de covarianzas Σ𝑢𝑡 ._

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
r.NewPlot(false, false, false);
%% 
% *Observaciones*
%% 
% * Según la pose odométrica, el robot describe un cuadrado perfecto. 
% * Sin embargo, la figura que describe el robot en la realidad dista mucho 
% de ser un cuadrado, y esto se debe a que los actuadores añaden ruido
% * El ruido en el ángulo tiene mucha influencia en el resultado final, ya que 
% a partir de ese punto los errores se propagan
% 1.a Implementación LSE
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
r.senRange=[100;2*pi]; % sensor sin limmitación
r.senSigma=[0.05;0.05]; % sensor muy preciso
for i = 1:44
    if mod(i,11) == 0
        r=r.Move([0;0;pi/2], false);
    else
        r=r.Move([1;0;0], false);
    end
end
r.NewPlot(true, false, false);
%% 
% *Observaciones*
%% 
% * La figura real que describe el robot es muy distinta a la del ejercicio 
% anterior porque al cambiar el número de balizas el generador de números aleatorios 
% ya no proporciona los mismos valores
% * Aunque la figura dista mucho de ser un cuadrado, la estimación de la posición 
% real es muy fiable
% * En la configuración del robot se ha establecido un rango de 100m y un ángulo 
% de 360º para que el sensor se comporte como si no tuviera limitaciones
% 1.b Implementación LSE
% _Vuelve a repetirlo con valores más altos (std_d=0.5)._

figure;
rng(seed); 
r=Robot(mapSize, 5, p0); % 5 landmarks
r.senRange=[100;2*pi]; % sensor sin limmitación
r.senSigma=[0.5;.5]; % sensor poco preciso
for i = 1:44
    if mod(i,11) == 0
        r=r.Move([0;0;pi/2], false);
    else
        r=r.Move([1;0;0], false);
    end
end
r.NewPlot(true, false, false);
%% 
% *Observaciones*
%% 
% * La figura real que describe el robot es idéntica al caso anterior porque 
% seguimos tuilizando la misma semilla.
% * Las estimaciones que realiza LSE son menos precisas, aunque siguen siendo 
% bastante fiables.
% 2. Ejercicio puntuable - LSE
% _Los sensores reales presentan ciertas limitaciones  físicas en cuanto al 
% rango y campo de visión. Modifica el ejercicio anterior para  contemplar que 
% el sensor sólo proporciona medidas en un rango limitado 𝑟𝑙 y una  orientación 
% limitada ±𝛼 con respecto a la pose del robot. Podría darse el caso que no  
% existieran landmarks en el campo de visión del sensor, con lo que el robot no 
% dispondría de información sensorial en una iteración._
% 
% _Aumenta el número de Landmarks a 10 y  considera el caso particular de un 
% sensor con  un rango máximo de 20m de alcance, y un FOV  de ±60º_
% _LSE con 3 o más balizas visibles_

figure;
rng(seed); 
r=Robot(mapSize, 10, p0); % 5 landmarks
r.senRange=[20;60*pi/180]; % rango de 20 metros y 60º
r.senSigma=[0.05;.05]; % sensor poco preciso
for i = 1:19
    if mod(i,11) == 0
        r=r.Move([0;0;pi/2], false);
    else
        r=r.Move([1;0;0], false);
    end
end
r.NewPlot(true, false, false);
%% 
% *Observaciones*
%% 
% * Anque el enunciado no lo especifica, hemos optado por versión más fiable 
% del sensor
% * Aunque las balizas son siempre las mismas, sólo se etiquetan las que están 
% en el rango de visión del robot.
% * En todo el trazado hasta el punto de la figura, la estimación se realiza 
% con 3 o más balizas a la vista, pero justo en ese momento el robot sólo tiene 
% a la vista las balizas L5 y L7.
% Tratamiento con menos de 3 balizas
% _Indica claramente en la memoria como tratas  el caso de no tener suficientes 
% observaciones,  y muéstralo en las gráficas de posición y de  error. Añade/explica 
% solo la parte del código  que has modificado._ 
% 
% Como con dos balizas no podemos utilizar LSE, partiendo de la última posición 
% estimada, reestimamos la posición componiendo la estimación anterior con la 
% acción odométrica

r=r.Move([1;0;0], false);
r.NewPlot(true, false, false);
%% 
% *Observaciones*
%% 
% * El robot real se ha desplazado hacia abajo, pero la estimación se mueve 
% hacia la izquierda. Esto es debido a que con LSE no tenemos implementado una 
% estimación del ángulo, así que es correcto en magnitud pero erróneo en dirección.
% Resultado final con LSE
% Completamos el cuadrado para evaluar el resultado final

for i = 21:44
    if mod(i,11) == 0
        r=r.Move([0;0;pi/2], false);
    else
        r=r.Move([1;0;0], false);
    end
end
r.NewPlot(true, false, false);
%% 
% *Observaciones*
%% 
% * Cada vez que el robot deja de tener 3 balizas a la vista, la estimación 
% "se pierde" porque mueve el robot en la dirección equivocada
% * La primera vez que ocurre, la estimación "camina" hacia la izquierda porque 
% el robot debería estar en la parte superior del cuadrado, aunque el desplazamiento 
% real es hacia abajo
% * Del mismo modo, cuando se vuelve a perder en el lado inferior del cuadrado 
% el robot sigue caminando hacia abajo porque en ese punto debería estar bajando 
% por el lado izquierdo del cuadrado
% * Cuando el robot recupera la visión de 3 o más balizas, la estimación se 
% corrige por completo
% 3 Implementación del filtro de partículas
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
% Esta funcionalidad está implementado en el método Move (ver apartado anterior). 
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
% 4.a Filtro sin correcciones
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
r=Robot(mapSize, 10, p0); % 5 landmarks
r.senRange=[20;60*pi/180]; % rango de 20 metros y 60º
r.senSigma=[0.05;.05]; % sensor poco preciso
for i = 1:44
    if mod(i,11) == 0
        r=r.Move([0;0;pi/2], false);
    else
        r=r.Move([1;0;0], false);
    end
end
r.NewPlot(false, true, true);
%% 
% *Observaciones*
%% 
% * Sólo se muestran las partículas del último paso, pero para ver la evolición 
% del movimento de las partículas se puede invocar al método de visualización 
% en cada paso.
% * La posición real del robot es idéntica a la del ejercicio anterior porque 
% en cada paso del robot se calculan todas las estimaciones y al final se decide 
% qué mostrar. 
% * Con una sola baliza elegida al azar, el filtro de partículas estima bastante 
% bien la posición del robot, y no tiene el problema de "desorientación" que observábamos 
% con LSE porque ahora tenemos una estimación del ángulo.
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
% a desviarse a la izquierda y este error se propaga hasta el final del cuadrado. 
% La media en este caso es de 5 metros, pero este valor depende mucho de la trayectoria 
% seguida
% * El error de la estimación con LSE tiene una media cercana a cero cuando 
% hay 3 o más balizas en el rango de visión, pero se dispara cada vez que el robot 
% "se pierde" en los pasos 19, 25 y 41. Aunque el error es bajo, este método no 
% permite hacer correcciones porque no hemos estimado el ángulo de la pose.
% * El error con la estimación del FP tiene un promedio de medio metro, que 
% aunque es bastante mayor al obtenido con LSE (se ha obtenido con una única baliza) 
% permite hacer correcciones porque disponemos de una estimación del ángulo de 
% la pose.
% 4.b Filtro con correcciones
% Capura de correcciones cada 5 pasos
% Luego añade otra captura donde se aprecien las correcciones cada 5 pasos  
% implementados en este ejercicio. 

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
    r.NewPlot(false, true, true);
    pause(0.5)
end
%% 
% *Observaciones*
%% 
% * Aunque el robot dibuja una figura muy distinta a un cuadrado, el resultado 
% se ajusta mejor que cuando no había correciones.
% * La mayor corrección se produce llegando a la esquina superior derecha del 
% cuadrado, y se aprecia que el robot vuelve a la posición odométrica con bastante 
% exactitud
%% 
% En este caso, muestra las gráficas de error  entre xEst y xOdom (para que 
% se aprecie que cada 5 pasos el error debe  hacerse muy pequeño), tanto de posición 
% como de orientación. 

r.PlotErrors(false, false, true, false);
%% 
% *Observaciones*
%% 
% * El error odométrico se ha reducido a menos de un metro porque las correciones 
% redirigen el robot a la posición requerida.
% * El error en la posición estimada por el filtro es del mismo orden de magnitud 
% que cuando no se hacían correciones.
% * El error en el ángulo se ha calculado como diferencia entre la pose real 
% del robot y la pose estimada. Hay oscilaciones significativas (cerca de +40º 
% en el paso 19 y -30º en el paso 4), pero en general los ángulos coinciden bastante 
% bien.