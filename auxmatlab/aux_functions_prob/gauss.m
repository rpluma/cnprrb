function y=gauss(mu,sigma,x)
% Computes the value of N(mu,sigma) for a given x
y = exp(-1*((x-mu).^2)/(2*sigma^2))/sqrt(2*pi*sigma^2);

end
