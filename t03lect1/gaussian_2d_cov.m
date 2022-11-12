function m=gaussian_2d_cov(mu, Sigma, size)
    r = mvnrnd(mu, Sigma, size);
    close all; figure(1); hold on;
    plot(r(:,1), r(:,2), 'b.');
end
