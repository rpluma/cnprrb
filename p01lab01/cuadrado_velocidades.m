function x = cuadrado_velocidades(lado)
   %clear all;
   close all;
   
   x = zeros(3, 1); % pose of the robot; initially at (0, 0, 0)
   DeltaT = 0.5; % Los comandos de velocidad se aplican cada 0.5s
   Vp = 1/DeltaT; % Velocidad lineal = espacio/tiempo
   W = (pi/2)/DeltaT; % Velocidad angular = ángulo/tiempo

   figure(1); hold on; axis equal; grid on;
   plot (-1, -1, 'bx'); plot (11, 11, 'bx')

   for i = 1:(4*(lado+1))
       plot(x(1), x(2), 'r.');
       pause(0.25);
       if mod(i, lado+1) == 0 
           u = [0, W]'; % actualización en esquinas con giro de 90º
       else
           u = [Vp, 0]'; % actualización en línea recta no tiene giro
       end

       if (u(2) == 0) % sin rotación
           ux = u(1)*DeltaT;
           uy=0; 
       else % resolvemos las integrales
           ux =(u(1)/u(2))*sin(u(2)*DeltaT); % Vp/W sin(w DeltaT)
           uy = (u(1)/u(2))*(1 - cos(u(2)*DeltaT)); % Vp/W (1 - cos(w DeltaT)
       end
       ut = u(2)*DeltaT; % W DeltaT
       x = pose_comp(x, [ux; uy; ut]);
   end
end