function y=sampling(mu,sigma,n)
% Produces n samples from a N(mu,sigma) usign the function randn
y=sigma*randn(n,1)+mu;
plot(y,0,'k.')

end
