% #8 The Gaussian Distribution 2D

clear;
mu = zeros(2, 1);
Sigma=[3 0;0 2]
SigmaInv = inv(Sigma);
x=[1;0]; 
Delta2 = (((x-mu)')*SigmaInv)*(x-mu) % escalar que define la elipse
clear;

mu = zeros(2, 1);
Sigma=[3 1;1 4]
SigmaInv = inv(Sigma);
x=[1;0]; 
Delta2 = (((x-mu)')*SigmaInv)*(x-mu) % escalar que define la elipse
clear;

%muestreo(100, 0, 1);

%close all;
%hist(randn(100));

%close all;
%hist(randn(1000));
%histogram(randn(100));

%gaussian_2d(100, 2, 1, 3, 2);
mu=[1 -1];
Sigma= [.9 0; 0 .3];
%Sigma=[.9 .4; .4 .3];
gaussian_2d_cov(mu', Sigma, 500);
PlotEllipse(mu,Sigma, 1)

