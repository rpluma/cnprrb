function x = cuadrado_ruido(lado)
   %clear all;
   close all;
   
   xOdom = zeros(3, 1); % pose of the robot; initially at (0, 0, 0)
   xTrue = zeros(3, 1); % true position of the robot, including noise
   u = zeros(3, 1); % control action, intitially set to (0, 0, 0)
   
   %% Noisy control action
   SigmaX = 0.07; SigmaY = 0.07; SigmaTheta= 0.0015;
   Q = diag([SigmaX^2 SigmaY^2 SigmaTheta^2]);

   figure(1); hold on; axis equal; grid on;
   plot (-1, -1, 'bx'); plot (11, 11, 'bx')

   for i = 1:(40*lado)
       plot(xOdom(1), xOdom(2), 'r.');
       if i <= (4*lado)
           plot(xTrue(1), xTrue(2), 'g.');
       else
           plot(xTrue(1), xTrue(2), 'b.');
       end
       

       pause(0.25);
       if mod(i, lado) == 0 
           u = [1 0 pi/2]';
       else
           u = [1 0 0]';
       end
       noise = sqrt(Q)*randn(3,1);
       uNoisy = pose_comp(u, noise);

       xOdom = pose_comp(xOdom, u);
       xTrue = pose_comp(xTrue, uNoisy);
   end
end