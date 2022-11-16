function plot2(mu1,sigma1,mu2,sigma2,n)
% Plots n samples from a 2D Gaussian Distribution
    x=sampling(mu1,sigma1,n);
    y=sampling(mu2,sigma2,n);
    plot(x,y,'k.')
end