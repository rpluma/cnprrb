% close all; plotea_normal(1, 0, 1); plotea_normal(-1, 2, 1); plotea_normal(-2, 0, 2);
function fx=plotea_normal(x, mu, sigma) 
    figure(1); hold on; grid on;
    plot(x, normal_density(x, mu, sigma), 'bx');
    for x=-5:0.1:5
        %fx = (1/(2*pi*sigma^2))*exp((-1/(2*sigma^2))*(x-mu)^2);
        plot(x, normal_density(x, mu, sigma), 'r.');
    end
end