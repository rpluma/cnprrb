function [handle]=plot_gauss(mu,sigma,width,inc)
% Plots the gaussian with a given mu and sigma for values in -width:inc:width
% Returns the handle of the plot  
x=-width:inc:width;
if (length(x)==0) disp('x vector is empty: Check the parameter width and inc') 
end
handle=plot(x,gauss(mu,sigma,x));
end