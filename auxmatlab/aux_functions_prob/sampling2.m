function y=sampling2(mu,sigma2,n)
% Produces n samples from a N(mu,sigma2) usign the function mvnrnd. Where
% sigma2 is the covariance matrix
y=mvnrnd(mu,sigma2,n);
plot(y,0,'k.');


end
