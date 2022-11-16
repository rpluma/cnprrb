%This code is based on the AckermannPredict.m and EKFSLAM files from Paul Newman
%and code by Peter Corke
%Example for the odometry motion model
%(c) Cipriano Galindo, Oct. 2013

clear all;
close all;


nSteps = 20;%length of run

SigmaX = 0.15; % sigma of the motion error in X

x = [0;0;0]; % initial pose
pose_sigma=0; %% initially we are sure where we are.
xtrue = x; % at the very beginning xtrue and odometry coincide
odometry=x;

Q = SigmaX^2;

%-------- Set up graphics -----------%
figure(1);hold on;axis equal;grid on;

xlabel('x');ylabel('y');
title('Uncertainty for Odometry 1D motion model (Odometry:red)');

%-------- Main loop -----------%

xTrueLast=x;   %%x_(t-1)
xnext = zeros(3,1); %some initializations
u=zeros(3,1);
len=100;
inc_x=1;
k=1;

grid on;
plot_heigth=5;
if (pose_sigma==0) 
    h1=line([0 0],[0 plot_heigth]);
end
    
while (k<nSteps)

    u(1)=inc_x;
    u(2)=0;
    u(3)=0;
    
    odometry = tcomp(odometry,u);       

    %including noise to model the ground truth position
    %noise based on the matrix Q (covariance matrix of the movement)
    noise = [sqrt(Q)*randn(1,1); 0; 0];
    
    %composing the control action with the noise:
    unoise=tcomp(u,noise);
    xTrueNow=tcomp(xTrueLast, unoise);
    plot(xTrueNow(1),xTrueNow(2),'b.');     
    xTrueLast=xTrueNow;
    %updating the uncertainty of the pose as convolution of gaussians
    pose_sigma=pose_sigma+sqrt(Q);
    pose_mu=odometry;
    %drawing
    h2=plot_gauss(pose_mu(1),pose_sigma,10+pose_mu(1),0.1);
    axis([-2 nSteps -2 plot_heigth])
    
    r=drawrobot(xTrueNow,'b');
    r1=drawrobot(odometry,'r');
    plot(odometry(1),0,'ro');
    
    k=k+1;
    pause;
    if (~isempty(r)) delete(r);end;
    if (~isempty(r1)) delete(r1);end;
    if (~isempty(h1)) delete(h1);h1=h2;end;
    
end
