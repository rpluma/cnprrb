function histogram(mu,sigma,n)
% Generates n random numbers under a normal distribution N(mu,sigma) and
% plot them using a histogram with a fix number of bins.

y=sigma*randn(n,1)+mu;
hist(y,10)
end