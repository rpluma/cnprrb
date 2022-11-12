% z=(x-mu)/sigma => x=z*sigma + mu

function m=muestreo(size, mu, sigma)
    z=randn(size);
    x=z*sigma + mu;
    close all; figure(1); hold on;
    plot(mu-6*sigma, 0, 'bx');
    plot(mu+6*sigma, 0, 'bx');
    for i=1:size
        plot(x(i), rand(1), 'r.');
    end

end
