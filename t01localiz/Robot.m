classdef Robot
	%properties (GetAccess = private)
	%	iHist
	%end

	properties (Constant)
		lseMax = 100; % Max iteration
		lseTol = 1.0e-09; % Tolerance
		fpPart = 100;
		fpMethod = 'weight'; % best, mean, weight
		maxHist = 100;

	end

	properties(Access = public)
		% Mapa y landmarks
			mapSize % tamaño del mapa
			ogrMap % objeto gráfico para el mapa
			numLmarks % número de balizas
			posLmarks % posición de las balizas
			ogrLmarks % objeto gráfico de cada baliza
			ogrLegend % lista de elementos en la leyenda

		% Poses e histórico de poses
			pTrue % posición real (sujeta a ruido)
			pOdom % posición odométrica (ideal sin ruido)
			pEstL % estimación LSE
			pEstF % estimación FP		
			pPart % pose de las partículas
			pHist % Histórico de poses (True+Odom+LSE+FP)
			bHist % Histórico de balizas (Visibles, Elegida, distancia)			
			iHist % Índice a arrays de históricos

		% Precición y rango de actuadores y sensores
			actSigma % desviación típica en [m m rad]
			senSigma % desviación típica en [m rad]'
			senRange % alcance (distancia/FOV) [m rad]'

		% lectura y visibilidad de balizas
			zTrue 		% distancia y ángulo real desde zTrue
			zNoisy 		% distinacia y ángulo desde zTrue con ruido
			bVisible	% balizas dentro del rango y distancia
			nVisible	% número de balizas en rango del sensor
			iVisible	% baliza elegida al azar
	end
	
	methods
		function obj = Robot(mapSize, numLmarks, pose0)
			% Mapa y landmarks
				obj.mapSize 	= mapSize;
				obj.ogrMap 		= 0;
				obj.numLmarks 	= numLmarks;
				obj.posLmarks 	= mapSize*rand(2, numLmarks)-mapSize/2;
				obj.ogrLmarks 	= gobjects(2, numLmarks);
				obj.ogrLegend   = 0;
			
			% Poses e histórico de poses
				obj.pTrue 		= pose0;
				obj.pOdom 		= pose0;
				obj.pEstL 		= pose0; 
				obj.pEstF 		= pose0; 
				obj.pPart 		= zeros(3, obj.fpPart) + pose0;

				obj.pHist		= zeros(12, obj.maxHist);
				obj.bHist		= zeros(3,  obj.maxHist); 
				obj.iHist		= 1;

				obj.pHist(:,obj.iHist) = [obj.pTrue; obj.pOdom; obj.pEstL; obj.pEstF];

			% Precición y rango de actuadores y sensores
				%obj.actSigma 	= [0.5 0.5 0.5*pi/180]'; 
				%obj.actSigma 	= [0.1 0.1 5*pi/180]'; 
				obj.actSigma 	= [0.2 0.2 10*pi/180]'; 
				%obj.senSigma 	= [0.05 0.5*pi/180]'; 
				%obj.senSigma 	= [0.1 5*pi/180]'; 
				obj.senSigma 	= [0.2 10*pi/180]'; 
				obj.senRange 	= [20 60*pi/180]'; 

			% lectura y visibilidad de balizas
				obj.zTrue 		= zeros(2, numLmarks);
				obj.zNoisy 		= zeros(2, numLmarks);
				obj.bVisible	= ones (1, numLmarks);
				obj.nVisible	= numLmarks;
				obj.iVisible	= 0;
		end % constructor

		function obj = Sense(obj)
    		for i = 1:obj.numLmarks
        		obj.zTrue(:, i) = dist_angle(obj.pTrue, obj.posLmarks(:, i));

		        obj.zNoisy(:, i) = obj.zTrue(:,i) + obj.senSigma .*randn(2, 1);
		        % TODO CONFIRMAR QUE AQUÍ NO DEPENDE DE LA DISTANCIA
	            % el ruido de la distancia es proporcional a su raiz cuadrada
        		%zNoisy(1,i) = zTrue(1:i)+Robot.senSigma(1)*randn(1,1)*sqrt(zTrue(1:i));
		        % el ruido del ángulo no depende de la diferencia de ángulos
        		%zNoisy(2,i) = zTrue(2:i)+Robot.senSigma(2)*randn(1,1);

        		% distancia <= rango AND angulo < FOV
        		obj.bVisible(i) = obj.zTrue(1,i)<=obj.senRange(1) && ...
        			(abs(obj.zTrue(2,i)) <= obj.senRange(2));        
    		end
    		obj.nVisible = sum(obj.bVisible);
    		obj.iVisible = randperm(obj.numLmarks, 1); % aleatorio
		end % sense

		function obj = LSE(obj, uOdom)
			if obj.nVisible < 3
				obj.pEstL = comp_odom(obj.pEstL, uOdom);
			else
				obj.pEstL = obj.pOdom;
			    iter = 0;
    			incr = ones(1,2);       % initialize increment (step)    
    			h = zeros(obj.nVisible, 1); % predicted observations (sensor model)
    			jH = zeros(obj.nVisible,2); % jacobian of the observation function for all the landmarks
    
			    % utilizar sólo información de los landmarks en rango
			    iVisible = find(obj.bVisible==1);
    			Landmarks=obj.posLmarks(:, iVisible);
    			zd = obj.zNoisy(1, iVisible); % usar sólo sensor de distancia
    			zd = zd'; % trasponer para que haya una lectura por fila
    
    			% LSE LOOP (iterative Method)
    			while (norm(incr)>obj.lseTol) && (iter<obj.lseMax)
        			iter = iter+1;        

        			% compute the prediction (from current pEstL) and build the Jacobian
        			for i = 1:obj.nVisible
            			% Expected distances between the map and the pose estimated
            			Delta = Landmarks(1:2,i)-obj.pEstL(1:2);  
            			h(i) = norm(Delta);            % predicted observation
            			% Jacobian evaluated in pEstL
            			jH(i,1) = -Delta(1) / h(i);
            			jH(i,2) = -Delta(2) / h(i);
        			end
    
        			% Para cada landmark, distancia leída vs distancia estimada
        			error = zd - h;   % difference between measurement and prediction

        			% observation variance grow with the root of the distance
        			R = diag(obj.senSigma(1)^2*sqrt(zd));
        
        			% incr!
        			incr = inv(jH'*inv(R)*jH) * jH'*inv(R)*error;
    
        			% Update Estimation pEstL
        			obj.pEstL(1:2) = obj.pEstL(1:2)+incr;
			    end % while 			
			end
		end % LSE

		function obj = FP(obj)
			% utilizamos la baliza elegida al azar
			landmark = obj.posLmarks(:, obj.iVisible);
			z = obj.zNoisy(: , obj.iVisible);
    		
    		% calculamos los pesos y los normalizamos
			W = zeros(1, obj.fpPart);
    		CovInv = inv(diag(obj.senSigma).^2);
    		for i = 1:obj.fpPart
        		zPred = dist_angle(obj.pPart(:,i), landmark);
        		% exp(-0.5*(zTrue-zPred)'*obj.sensorCovInv*(zTrue-zPred))+0.01;
        		W(i) = 1/((z-zPred)'*CovInv*(z-zPred)+0.001);
    		end
    		W = W/sum(W);

    		% seleccionamos las mejores partículas
		    CDF=cumsum(W)/sum(W);
    		iSelect=rand(obj.fpPart,1);
    		iNext=interp1(CDF,1:obj.fpPart,iSelect,'nearest','extrap');
    		obj.pPart = obj.pPart(:, iNext); 
    
    		% estimamos según el método elegido
    		if obj.fpMethod == "best" % pose de la mejor partícula
        		[wMax, iMax] = max(W);
        		obj.pEstF = obj.pPart(:, iMax); 
    		elseif obj.fpMethod == "mean" % media de poses de partículas
    			obj.pEstF(1) = sum(obj.pPart(1, :));
        		obj.pEstF(2) = sum(obj.pPart(2, :));        
        		obj.pEstF(3) = circ_mean(obj.pPart(3, :)');
        	elseif obj.fpMethod == "weight" % media ponderada de poses
				obj.pEstF(1) = sum(obj.pPart(1, :).*W);
        		obj.pEstF(2) = sum(obj.pPart(2, :).*W);
        		obj.pEstF(3) = circ_mean(obj.pPart(3, :)', W');	
    		end
			
		end

		function obj = Plot(obj, iOrg, iDst, bLSE, bFP, bPart)
			% inicialización del mapa
			if (obj.ogrMap==0)
				obj.ogrMap = true;
				figure;
				set(gcf,'Visible','on');    % pup-up window
				mg = obj.mapSize*0.5;
				x0 = obj.pTrue(1);
				y0 = obj.pTrue(2);

				% primer objeto de cada tipo para la leyenda
				l1= plot([-mg mg mg -mg -mg], [-mg -mg mg mg, -mg], '--r', 'DisplayName','Room');
				hold on; grid on; axis equal;				
				% obj.ogrLmarks(1,1) = plot(obj.posLmarks(1,1),obj.posLmarks(2,1),'sm', 'DisplayName','Landmark');
				l3=plot([x0 x0], [y0 y0], 'kx-','LineWidth',2, 'DisplayName','True position');
				l4=plot([x0 x0], [y0 y0], 'go-','LineWidth',2, 'DisplayName','LSE Estimation');
				l5=plot([x0 x0], [y0 y0], 'mo-','LineWidth',2, 'DisplayName','LSE <3 landmarks');
				l6=plot([x0 x0], [y0 y0], 'co-','LineWidth',2, 'DisplayName','FP Estimation');
			    % obj.ogrLegend = [l1, obj.ogrLmarks(1,1), l3, l4, l5, l6];

			    % resto de landmarks
			    %obj.ogrLmarks(2,1) = text(obj.posLmarks(1,1)+1,obj.posLmarks(2,1), sprintf('L%d',1) );
			    for k = 1:obj.numLmarks
			        obj.ogrLmarks(1,k) = plot(obj.posLmarks(1,k),obj.posLmarks(2,k),'sm', 'DisplayName','Landmark');
			        obj.ogrLmarks(2,k) = text(obj.posLmarks(1,k)+1,obj.posLmarks(2,k), sprintf('L%d',k) );
			    end
			end

			% Visualización de balizas
			for i=1:obj.numLmarks
				% oculta las etiquetas de las balizas no visibles para LSE
        		obj.ogrLmarks(2,i).Visible = obj.bVisible(i);
        		% resalta la baliza elegida al azar para el FP
        		obj.ogrLmarks(1,i).LineWidth=2+2*(obj.iVisible==i);
        		%end
    		end
    		
    		% Trazado de movimiento de las poses True/Odom
    		for i=iOrg:(iDst-1)
    			if bLSE % Visualizar las estimaciones del LSE
    				if obj.bHist(1,i) >= 3 % >= 3 balizas => estimación LSE verde
    					plot(obj.pHist(7,i:i+1), obj.pHist(8,i:i+1), 'go-','LineWidth',2); % LSE
    				else % < 3 => estimación no LSE (rojo)
    					plot(obj.pHist(7,i:i+1), obj.pHist(8,i:i+1), 'mo-','LineWidth',2); % <3 balizas
    				end
    			end
    			if bPart % visualizar las partículas individuales
			        part=plot(obj.pPart(1, :), obj.pPart(2, :), '.');
			        set(part, 'MarkerSize', 5, 'Color', [rand rand rand]);
				end
    			if bFP % visualizar las estimaciones del FP
					plot(obj.pHist(10,i:i+1), obj.pHist(11,i:i+1), 'co-','LineWidth',2); % FP
    			end

    			plot(obj.pHist(1,i:i+1), obj.pHist(2,i:i+1), 'kx-'); % True	
    			plot(obj.pHist(4,i:i+1), obj.pHist(5,i:i+1), 'b+-'); % Odom
    		end
    		
		end

		function obj = Move(obj,uOdom)
			obj.iHist = obj.iHist + 1;

			% mueve las poses real, odométrica y de las partículas
			obj.pTrue = comp_noisy(obj.pTrue, uOdom, obj.actSigma);
		    obj.pOdom = comp_odom(obj.pOdom, uOdom);
		    obj.pPart = comp_noisy(obj.pPart, uOdom, obj.actSigma);

		    % lee el sensor para cada baliza y actualiza el histórico
		    obj = obj.Sense();
		    
		    % estima la posición a partir de las lecturas del sensor
		    obj = obj.LSE(uOdom);
		    obj = obj.FP();

		    % actualiza el histórico de poses
		    obj.pHist(:,obj.iHist) = ... % poses
		    	[obj.pTrue; obj.pOdom; obj.pEstL;obj.pEstF];    		
		    
		    % actualiza el histórico de balizas
		    obj.bHist(:,obj.iHist) = ... % num visibles, elegida, distancia
		    	[sum(obj.bVisible); obj.iVisible; obj.zTrue(1, obj.iVisible)];
		end % Move

		function obj = Fix(obj)
			obj.iHist = obj.iHist + 1;

	        delta = dist_angle(obj.pEstF, obj.pOdom);
	        %F0 = obj.pEstF
	        %O0 = obj.pOdom
	        %T0 = obj.pTrue
	        %D0 = (obj.pOdom-obj.pEstF)
        	line([obj.pEstF(1), obj.pOdom(1)], [obj.pEstF(2), obj.pOdom(2)],...
        		'LineStyle',':', 'Color','red', 'LineWidth',2);
                    
	        % giro del robot y partículas hacia la posición odométrica
	        u_giro1 	= [0 0 delta(2)]'; % correción ángulo hacia odométrica
	        obj.pTrue 	= comp_noisy(obj.pTrue, u_giro1, obj.actSigma);
	        obj.pPart 	= comp_noisy(obj.pPart, u_giro1, 0); % obj.actSigma);
	        %obj.pEstL 	= comp_odom(obj.pEstL, u_giro1);
	        obj.pEstF 	= comp_odom(obj.pEstF, u_giro1);
        
	        % avance hasta la posición odométrica
	        u_avanc 	= [delta(1) 0 0]'; % avance hasta odométrica
	        obj.pTrue 	= comp_noisy(obj.pTrue, u_avanc, obj.actSigma);
	        obj.pPart 	= comp_noisy(obj.pPart, u_avanc, 0); % obj.actSigma);
	        %obj.pEstL 	= comp_odom(obj.pEstL, u_avanc);
	        obj.pEstF 	= comp_odom(obj.pEstF, u_avanc);
        
	        % giro hacia la pose odométrica
	        u_giro2 	= [0 0 obj.pOdom(3)-obj.pEstF(3)]';
	        obj.pTrue 	= comp_noisy(obj.pTrue, u_giro2, obj.actSigma);
	        obj.pPart 	= comp_noisy(obj.pPart, u_giro2, 0); % obj.actSigma);
	    	%obj.pEstL	= comp_odom(obj.pEstL, u_giro2);
	        obj.pEstF 	= comp_odom(obj.pEstF, u_giro2);

	        obj = obj.FP();

	        %F1 = obj.pEstF
	        %O1 = obj.pOdom
	        %T1 = obj.pTrue
	        %D1 = (obj.pOdom-obj.pEstF)

	        % movemos todas las estimaciones ajustando la diferencia
	        %delta = (obj.pOdom-obj.pEstF);
	        %obj.pEstL = obj.pEstL+(obj.pOdom-obj.pEstF);
	        %obj.pPart = obj.pPart+(obj.pOdom-obj.pEstF);
	        
	        % estimamos la nueva posición LSE y FP
	        %obj = obj.LSE(uOdom);
	        %obj = obj.Sense();
	        %obj = obj.FP();

	        % estimamos la posición mediante filtro de partículas
	        %[xEst, Best, bVisible] = est_fp(Robot, Mapa, zNoisy, 1);
	        %obj.pEstF = xEst; % nueva posición del robot
	        %obj.pPart = Best; % nueva posición de partículas seleccionadas

		    % actualiza el histórico de poses
		    obj.pHist(:,obj.iHist) = ... % poses
		    	[obj.pTrue; obj.pOdom; obj.pEstL;obj.pEstF];    		
		    
		    % actualiza el histórico de balizas
		    obj.bHist(:,obj.iHist) = ... % num visibles, elegida, distancia
		    	[sum(obj.bVisible); obj.iVisible; obj.zTrue(1, obj.iVisible)];

		end

		function PlotErrors(obj)
			iMax = obj.iHist;
			rng = 1:obj.iHist;

			% variables para errores en distancia Odométrico, Lse y Fp
			errDistO = zeros(1, iMax);
			errDistL = zeros(1, iMax);
			errDistF = zeros(1, iMax);

			% variables para errores en ángulos Odométrico y Fp
			errAnguO = zeros(1, iMax);
			errAnguF = zeros(1, iMax);

			% cálculo de errores den distancias y ángulos
			for i=1:obj.iHist
			    errDistO(i) = norm(obj.pHist(1,i)-obj.pHist(4,i),obj.pHist(2,i)-obj.pHist(5,i));
			    errDistL(i) = norm(obj.pHist(1,i)-obj.pHist(7,i),obj.pHist(2,i)-obj.pHist(8,i));
			    errDistF(i) = norm(obj.pHist(1,i)-obj.pHist(10,i),obj.pHist(2,i)-obj.pHist(11,i));

			    errAnguO(i) = angle_sum(obj.pHist(3,i), -obj.pHist(6,i))*180/pi;
			    errAnguF(i) = angle_sum(obj.pHist(3,i), -obj.pHist(12,i))*180/pi;
			end

			% errores en distancia
			figure; hold on; set(gcf, 'Visible', 'on');
			subplot(311); 
			plot(errDistO,'b');m=mean(errDistO);line([1 iMax], [m, m]);
			title('Odometric Error - Distance');
			subplot(312); 
			plot(errDistL,'g');m=mean(errDistL);line([1 iMax], [m, m]);
			title('LSE Error - Distance');
			subplot(313); 
			plot(errDistF,'c');m=mean(errDistF);line([1 iMax], [m, m]);
			title('FP Error - Distance');

			% detalle LSE
			figure; hold on; set(gcf, 'Visible', 'on');
			subplot(311);
			plot(errDistO,'b');m=mean(errDistO);line([1 iMax], [m, m]);
			title('Odometric Error - Distance');
			subplot(312);
			plot(errDistL,'g');m=mean(errDistL);line([1 iMax], [m, m]);
			title('LSE Error - Distance');
			subplot(313);
			plot(obj.bHist(1,rng),'r');m=2.5;line([1 iMax], [m, m]);
			title('LSE Error - # Visible sensors');

			% detalle FP
			figure; hold on; set(gcf, 'Visible', 'on');
			subplot(311);
			plot(errDistO,'b');m=mean(errDistO);line([1 iMax], [m, m]);
			title('Odometric Error - Distance');
			subplot(312);
			plot(errDistF,'c');m=mean(errDistF);line([1 iMax], [m, m]);
			title('FP Error - Distance');
			subplot(313);
			plot(obj.bHist(3,rng),'r');m=mean(obj.bHist(3,rng));line([1 iMax], [m, m]);
			title('FP Error - # Distance to random landmark');

			% errores en ángulos
			figure; hold on; set(gcf, 'Visible', 'on');
			subplot(311); 
			plot(obj.pHist(6, rng)*180/pi,'b');
			title('Odometric angle (Desired)');
			subplot(312); 
			plot(errAnguO,'b');m=mean(errAnguO);line([1 iMax], [m, m]);
			title('Odometric angleº error (True - Desired)');
			subplot(313); 
			plot(errAnguF,'c');m=mean(errAnguF);line([1 iMax], [m, m]);
			title('FP angleº error (True - Estimated)');

		end

	end % methods
end % classdef

