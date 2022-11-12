function m=gaussian_2d(size, mux, sigmax, muy, sigmay)
    zx=randn(size);
    zy=randn(size);

    x=zx*sigmax + mux;
    y=zy*sigmay + muy;
    close all; figure(1); hold on;
    lado = max(abs(mux)+3*sigmax, abs(muy)+3*sigmay);
    plot(-lado, -lado, 'bx');
    plot(+lado, +lado, 'bx');
    plot(x, y, 'r.');
    plot(mux, muy,'bo');    
end
