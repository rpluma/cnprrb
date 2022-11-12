function fx=normal_density(x, mu, sigma) 
    fx = (1/(2*pi*sigma^2))*exp((-1/(2*sigma^2))*(x-mu)^2);
end