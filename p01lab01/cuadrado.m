function x = cuadrado(lado)
   %clear all;
   close all;
   
   x = zeros(3, 1); % pose of the robot; initially at (0, 0, 0)
   u = zeros(3, 1); % control action, intitially set to (0, 0, 0)

   figure(1);
   hold on;
   axis equal;
   grid on;
   plot (-1, -1, 'bx')
   plot (11, 11, 'bx')

   for i = 1:(4*lado)
       plot(x(1), x(2), 'r.');
       pause(0.25);
       if mod(i, lado) == 0 
           u = [1 0 pi/2]';
       else
           u = [1 0 0]';
       end
       x = pose_comp(x, u);
   end
end