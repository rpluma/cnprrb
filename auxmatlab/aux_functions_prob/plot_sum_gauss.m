function plot_sum_gauss(mu1,sigma1,mu2,sigma2,width,inc)
% Plots the 2 gaussian with a given mu and sigma for values in 
% -width:inc:width along with their sum

x=-width:inc:width;
if (length(x)==0) disp('x vector is empty: Check the parameter width and inc') 
end
hold on;
plot(x,gauss(mu1,sigma1,x),'r')
plot(x,gauss(mu2,sigma2,x),'b')
plot(x,gauss(mu1+mu2,sigma1+sigma2,x),'k')
end