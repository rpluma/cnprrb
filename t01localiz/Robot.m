classdef Robot
	%properties (GetAccess = private)
	%	iHist
	%end

	properties (Constant)
		lseMax        = 100;      % LSE Max iteration
		lseTol        = 1.0e-09;  % LS Tolerance
		fpPart        = 100;      % Number of particles
		fpMethod      = 'weight'; % Estimation method: best, mean, weight
		maxHist       = 100;      % Max number of movements
	  end % constant prperties

	properties(Access = public)
		% Mapa y landmarks
			mapSize    % tamaño del mapa
			ogrMap     % objeto gráfico para el mapa
			numLmarks  % número de balizas
			posLmarks  % posición de las balizas
			ogrLmarks  % objeto gráfico de cada baliza
			ogrLegend  % lista de elementos en la leyenda

		% Poses e histórico de poses
			pTrue      % posición real (sujeta a ruido)
			pOdom      % posición odométrica (ideal sin ruido)
			pEstL      % estimación LSE
			pEstF      % estimación FP		
			pPart      % pose de las partículas
			pHist      % Histórico de poses (True+Odom+LSE+FP)
			bHist      % Histórico de balizas (Visibles, Elegida, distancia)			
			fHist      % Histórico de correcciones de posición
      iHist      % Índice a arrays de históricos

		% Precición y rango de actuadores y sensores
			actSigma   % desviación típica en [m m rad]
			senSigma   % desviación típica en [m rad]'
			senRange   % alcance (distancia/FOV) [m rad]'

		% lectura y visibilidad de balizas
			zTrue      % distancia y ángulo real desde zTrue
			zNoisy     % distinacia y ángulo desde zTrue con ruido
			bVisible   % balizas dentro del rango y distancia
			nVisible   % número de balizas en rango del sensor
			iLmFP      % baliza elegida al azar para el FP
	
    end % properties
	
	methods
		function r = Robot(mapSize, numLmarks, pose0)
    % Construct a robot and generates random landmarks

			% Mapa y landmarks
				r.mapSize 	= mapSize;
				r.ogrMap    = 0;
				r.numLmarks = numLmarks;
				r.posLmarks = mapSize*rand(2, numLmarks)-mapSize/2;
				r.ogrLmarks = gobjects(1, numLmarks);
			
			% Poses 
				r.pTrue 		= pose0;
				r.pOdom 		= pose0;
				r.pEstL 		= pose0; 
				r.pEstF 		= pose0; 
				r.pPart 		= zeros(3, r.fpPart) + pose0;

			% Precición y rango de actuadores y sensores
				r.actSigma 	= [0.2;0.2;10*pi/180]; % [0.5;0.5;0.5*pi/180]
				r.senSigma 	= [0.2;    10*pi/180]; % [0.05;0.5*pi/180]
				r.senRange 	= [20 ;    60*pi/180]; 

			% lectura y visibilidad de balizas
				r.zTrue 		= zeros(2, numLmarks);
				r.zNoisy 		= zeros(2, numLmarks);
				r.bVisible	= ones (1, numLmarks); % qué balizas son visibles
				r.nVisible	= numLmarks;           % número de balizas visibles
				r.iLmFP	= 0;                   % baliza elegida para FP

      % Históricos de poses y de sensores
        r.pHist     = zeros(12, r.maxHist);
        r.bHist     = zeros(3,  r.maxHist);
        r.fHist     = zeros(3,  r.maxHist);
        r.pHist(:,1)= [r.pTrue; r.pOdom; r.pEstL; r.pEstF];
        r.iHist     = 1;

		  end % constructor

		function r = Sense(r)
    % Computes distance and angle to each landmark

  		for i = 1:r.numLmarks
        % TODO CONFIRMAR QUE LA LECTURA NO DEPENDE DE LA DISTANCIA
        % el ruido de la distancia es proporcional a su raiz cuadrada
        % zNoisy(1,i) = zTrue(1:i)+Robot.senSigma(1)*randn(1,1)*sqrt(zTrue(1:i));
        % el ruido del ángulo no depende de la diferencia de ángulos
        %zNoisy(2,i) = zTrue(2:i)+Robot.senSigma(2)*randn(1,1);
    		r.zTrue(:, i) = dist_angle(r.pTrue, r.posLmarks(:, i));
        r.zNoisy(:, i) = r.zTrue(:,i) + r.senSigma .*randn(2, 1);
    		% distancia <= rango AND angulo < FOV
    		r.bVisible(i) = r.zTrue(1,i)<=r.senRange(1) && ...
                  			(abs(r.zTrue(2,i)) <= r.senRange(2));        
  		  end

      % cuántos son visibles para LSE y cuál se usará en FP
  		r.nVisible = sum(r.bVisible);
  		r.iLmFP = randperm(r.numLmarks, 1); % aleatorio

		  end % sense

		function r = LSE(r, uOdom)
    % Estimate position with Least Squares

      % Move the estimated possition according to last action
      r.pEstL = comp_odom(r.pEstL, uOdom);
			
      if r.nVisible >= 3 % LSE only works if 3 or more visible landmarks
        % initializations
		    iter  = 0;
    		incr  = ones(1,2);                  % initialize increment (step)    
    		h     = zeros(r.nVisible, 1);       % predicted observations (sensor model)
    		jH    = zeros(r.nVisible,2);        % jacobian of the observation     
        iRng  = find(r.bVisible==1);        % índice de balizas en rango
        lmRng = r.posLmarks(:, iRng);       % posiciones en rango
  			zdRng = r.zNoisy(1, iRng)';         % distancias en rango
  			
  			% LSE LOOP (iterative Method)
  			while (norm(incr)>r.lseTol) && (iter<r.lseMax)
    			
          % compute the prediction and build the Jacobian
    			for i = 1:r.nVisible
      			Delta = lmRng(1:2,i)-r.pEstL(1:2); % landmark(i) - estimate
      			h(i) = norm(Delta);              % predicted distance
      			jH(i,1) = -Delta(1) / h(i);      % jacobbian in x direction
      			jH(i,2) = -Delta(2) / h(i);      % jacobbian in y direction
      			end
    
    			% Updates
            R = diag(r.senSigma(1)^2*sqrt(zdRng)); % TODO USAR ZTRUE, MOVER A SENSE
            error = zdRng - h;  % difference between measurement and prediction
      			incr = inv(jH'*inv(R)*jH) * jH'*inv(R)*error; % matrix algebra
      			r.pEstL(1:2) = r.pEstL(1:2)+incr; % update estimate
            iter = iter+1;

			     end % while 			

			   end % if 3 or more landmarks

		  end % LSE

		function r = FP(r)
    % Estimate position with Filter of Particles

			% utilizamos la baliza elegida al azar
  			slm = r.posLmarks(:, r.iLmFP); 
  			z = r.zNoisy(: , r.iLmFP);
    		
    	% calculamos los pesos y los normalizamos
  			W = zeros(1, r.fpPart);
    		CovInv = inv(diag(r.senSigma).^2);
    		for i = 1:r.fpPart
      		zPred = dist_angle(r.pPart(:,i), slm);
      		% exp(-0.5*(zTrue-zPred)'*r.sensorCovInv*(zTrue-zPred))+0.01;
      		W(i) = 1/((z-zPred)'*CovInv*(z-zPred)+0.001);
      		end
    		W = W/sum(W);

  		% seleccionamos las mejores partículas
  	    CDF=cumsum(W)/sum(W);
    		iSelect=rand(r.fpPart,1);
    		iNext=interp1(CDF,1:r.fpPart,iSelect,'nearest','extrap');
    		r.pPart = r.pPart(:, iNext); 
    
    	% estimamos según el método elegido
        % pose de la mejor partícula
    		if r.fpMethod == "best" 
      		[wMax, iMax] = max(W);
      		r.pEstF = r.pPart(:, iMax); 

        % media de poses de partículas
    		elseif r.fpMethod == "mean" 
    			r.pEstF(1) = sum(r.pPart(1, :));
      		r.pEstF(2) = sum(r.pPart(2, :));        
      		r.pEstF(3) = circ_mean(r.pPart(3, :)');

        % media ponderada de poses
        elseif r.fpMethod == "weight" 
	    		r.pEstF(1) = sum(r.pPart(1, :).*W);
      		r.pEstF(2) = sum(r.pPart(2, :).*W);
      		r.pEstF(3) = circ_mean(r.pPart(3, :)', W');	
    		  end	

		  end % FP

    function r = NewPlot(r, bLSE, bFP, bPart)
      % Visualizar la habitación
        hold off
        mg = r.mapSize*0.75;
        x0 = r.pTrue(1);
        y0 = r.pTrue(2);
        plot([-mg mg mg -mg -mg], [-mg -mg mg mg, -mg], '--r', 'DisplayName','Room');
        hold on

      % Visualizar las balizas
        hold on; grid on; axis equal;       
        plot(r.posLmarks(1,:),r.posLmarks(2,:),'dr', 'DisplayName','Landmark');
        for k = 1:r.numLmarks
            r.ogrLmarks(1,k) = text(r.posLmarks(1,k)+1,r.posLmarks(2,k), sprintf('L%d',k) );
        end

      % Movimiento real y odométricos
        iRng = 1:r.iHist;
        plot(r.pHist(1,iRng), r.pHist(2,iRng), 'kp-', 'DisplayName','True position');
        plot(r.pHist(4,iRng), r.pHist(5,iRng), 'b.-', 'DisplayName','Odomtriy');

      % Estimaciones LSE y FP
      if bLSE 
          plot(r.pHist(7,iRng), r.pHist(8,iRng), 'mo','DisplayName','LSE');
          for i=1:r.numLmarks
            r.ogrLmarks(1,i).Visible = r.bVisible(i);
            end      
        end

      if bFP
        plot(r.pHist(10,iRng), r.pHist(11,iRng),'co','DisplayName','FP');
        end

      if bPart
        part=plot(r.pPart(1, :), r.pPart(2, :), '.','DisplayName','Particles');
        set(part, 'MarkerSize', 5, 'Color', [rand rand rand]);
        end  

      legend
        
        %a = annotation('arrow') ;
        %a.Parent = gca; 
        %a.Position = [-10,2,2,2];
      end % NewPlot
		
    function r = Plot(r, iOrg, iDst, bLSE, bFP, bPart)
			% inicialización del mapa
			if (r.ogrMap==0)
				r.ogrMap = true;
				figure;
				set(gcf,'Visible','on');    % pup-up window
				mg = r.mapSize*0.5;
				x0 = r.pTrue(1);
				y0 = r.pTrue(2);

				% primer reto de cada tipo para la leyenda
				plot([-mg mg mg -mg -mg], [-mg -mg mg mg, -mg], '--r', 'DisplayName','Room');
				hold on; grid on; axis equal;				
				plot(r.posLmarks(1,:),r.posLmarks(2,:),'sm', 'DisplayName','Landmark');
		    for k = 1:r.numLmarks
		        r.ogrLmarks(1,k) = text(r.posLmarks(1,k)+1,r.posLmarks(2,k), sprintf('L%d',k) );
		    end
			end

			% Ocultar las etiquetas de las balizas no visibles para LSE
			for i=1:r.numLmarks
				r.ogrLmarks(1,i).Visible = r.bVisible(i);
        end
    	
      % Origen y destino del movimiento de partículas
      o=r.iHist-1; d=r.iHist; 
      if o < 1
        o=1
      end

      % Visualizar las estimaciones del LSE con 3 o más balizas
      %if bLSE && r.bHist(1,d) >= 3
      %  plot(r.pHist(7,o:d), r.pHist(8,o:d), 'go-','LineWidth',2); % LSE

    		% Trazado de movimiento de las poses True/Odom
    		for i=iOrg:(iDst-1)
    			if bLSE 
    				if r.bHist(1,i) >= 3 % >= 3 balizas => estimación LSE verde
    					
    				else % < 3 => estimación no LSE (rojo)
    					plot(r.pHist(7,i:i+1), r.pHist(8,i:i+1), 'mo-','LineWidth',2); % <3 balizas
    				end
    			end
    			if bPart % visualizar las partículas individuales
			        part=plot(r.pPart(1, :), r.pPart(2, :), '.');
			        set(part, 'MarkerSize', 5, 'Color', [rand rand rand]);
				end
    			if bFP % visualizar las estimaciones del FP
					plot(r.pHist(10,i:i+1), r.pHist(11,i:i+1), 'co-','LineWidth',2); % FP
    			end

    			plot(r.pHist(1,i:i+1), r.pHist(2,i:i+1), 'kx-'); % True	
    			plot(r.pHist(4,i:i+1), r.pHist(5,i:i+1), 'b+-'); % Odom
    		end
    		
		end

		function r = Move(r, uOdom, bFix)
    % Moves the robot, fix if requested and update estimates
			
			% mueve las poses real, odométrica y de las partículas
        r.pTrue = comp_noisy(r.pTrue, uOdom, r.actSigma);
		    r.pOdom = comp_odom(r.pOdom, uOdom);
		    r.pPart = comp_noisy(r.pPart, uOdom, r.actSigma);

	    % lee el sensor y actualiza las estimaciones
		    r = r.Sense();
		    r = r.LSE(uOdom);
		    r = r.FP();

	    % actualiza los históricos de posiciones y balizas
		    r.iHist = r.iHist + 1;
        r.pHist(:,r.iHist) = [r.pTrue; r.pOdom; r.pEstL;r.pEstF];
		    r.bHist(:,r.iHist) = ... 
		    	[sum(r.bVisible); r.iLmFP; r.zTrue(1, r.iLmFP)];
		  
      % corrige la posición moviendo hacia la posición odométrica
      if bFix
        % diferencia a posición requerida y cómo llegar a ella
          delta     = dist_angle(r.pEstF, r.pOdom);
          u_giro1   = [0;0;delta(2)]; % ángulo hacia odométrica
          r.pEstF   = comp_odom(r.pEstF, u_giro1);
          u_avanc   = [delta(1);0;0]; % avance hasta odométrica
          r.pEstF   = comp_odom(r.pEstF, u_avanc);
          u_giro2   = [0;0;r.pOdom(3)-r.pEstF(3)]; % corrección ángulo
          r.pEstF   = comp_odom(r.pEstF, u_giro2);
                    
        % mover el robot y volver a sensar
          r.pTrue = comp_noisy(r.pTrue, u_giro1, r.actSigma);          
          r.pTrue = comp_noisy(r.pTrue, u_avanc, r.actSigma);          
          r.pTrue = comp_noisy(r.pTrue, u_giro2, r.actSigma);
          r       = r.Sense();
        
        % mover las partículas sin ruido (desv típica 0) 
          r.pPart   = comp_odom(r.pPart, u_giro1);% , 0); % r.actSigma);
          r.pPart   = comp_odom(r.pPart, u_avanc);% , 0); % r.actSigma);
          r.pPart   = comp_odom(r.pPart, u_giro2);% , 0); % r.actSigma);

        % reestimar posición con FP
          %r = r.FP();

        % actualiza los históricos de posiciones y balizas
          r.fHist(r.iHist) = 1;  % el siguiente paso es una corrección
          r.iHist = r.iHist + 1;
          r.pHist(:,r.iHist) = [r.pTrue; r.pOdom; r.pEstL;r.pEstF];
          r.bHist(:,r.iHist) = ... 
            [sum(r.bVisible); r.iLmFP; r.zTrue(1, r.iLmFP)];
        end % corregir posición

    end % Move


		function PlotErrors(r, bDist, bLSE, bFP, bAng)
			iMax = r.iHist;
			rng = 1:r.iHist;

			% variables para errores en distancia Odométrico, Lse y Fp
			errDistO = zeros(1, iMax);
			errDistL = zeros(1, iMax);
			errDistF = zeros(1, iMax);

			% variables para errores en ángulos Odométrico y Fp
			errAnguO = zeros(1, iMax);
			errAnguF = zeros(1, iMax);

			% cálculo de errores den distancias y ángulos
			for i=1:r.iHist
			    errDistO(i) = norm(r.pHist(1,i)-r.pHist(4,i),r.pHist(2,i)-r.pHist(5,i));
			    errDistL(i) = norm(r.pHist(1,i)-r.pHist(7,i),r.pHist(2,i)-r.pHist(8,i));
			    errDistF(i) = norm(r.pHist(1,i)-r.pHist(10,i),r.pHist(2,i)-r.pHist(11,i));

			    errAnguO(i) = angle_sum(r.pHist(3,i), -r.pHist(6,i))*180/pi;
			    errAnguF(i) = angle_sum(r.pHist(3,i), -r.pHist(12,i))*180/pi;
			end

			% errores en distancia
      if bDist
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
      end

			% detalle LSE
      if bLSE
  			figure; hold on; set(gcf, 'Visible', 'on');
  			subplot(311);
  			plot(errDistO,'b');m=mean(errDistO);line([1 iMax], [m, m]);
  			title('Odometric Error - Distance');
  			subplot(312);
  			plot(errDistL,'g');m=mean(errDistL);line([1 iMax], [m, m]);
  			title('LSE Error - Distance');
  			subplot(313);
  			plot(r.bHist(1,rng),'r');m=2.5;line([1 iMax], [m, m]);
  			title('LSE Error - # Visible sensors');
      end

			% detalle FP
      if bFP
  			figure; hold on; set(gcf, 'Visible', 'on');
  			subplot(311);
  			plot(errDistO,'b');m=mean(errDistO);line([1 iMax], [m, m]);
  			title('Odometric Error - Distance');
  			subplot(312);
  			plot(errDistF,'c');m=mean(errDistF);line([1 iMax], [m, m]);
  			title('FP Error - Distance');
  			subplot(313);
  			%plot(r.bHist(3,rng),'r');m=mean(r.bHist(3,rng));line([1 iMax], [m, m]);
  			%title('FP Error - # Distance to random landmark');
        plot(errAnguF,'c');m=mean(errAnguF);line([1 iMax], [m, m]);
        title('FP angleº error (True - Estimated)');
      end

			% errores en ángulos
      if bAng
  			figure; hold on; set(gcf, 'Visible', 'on');
  			subplot(311); 
  			plot(r.pHist(6, rng)*180/pi,'b');
  			title('Odometric angle (Desired)');
  			subplot(312); 
  			plot(errAnguO,'b');m=mean(errAnguO);line([1 iMax], [m, m]);
  			title('Odometric angleº error (True - Desired)');
  			subplot(313); 
  			plot(errAnguF,'c');m=mean(errAnguF);line([1 iMax], [m, m]);
  			title('FP angleº error (True - Estimated)');
      end

		end

	end % methods
end % classdef

