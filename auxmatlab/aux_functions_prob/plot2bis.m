function plot2bis(mu,sigma,n)
% Plots n samples from a 2D Gaussian Distribution using the mvnrnd function
y=mvnrnd(mu,sigma,n);
plot(y(:,1),y(:,2),'ko')
end