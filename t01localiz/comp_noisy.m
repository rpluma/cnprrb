function [mDest] = comp_noisy(mOrig, cUpdate, cSigma)
%COMP_NOSIY Compone una pose y actualización con ruido
%   mOrig es una matriz 3xn con la(s) pose(s) de origen
%   cUpdate es un vector columna con la actualización a realizar
%   cSigma es el vector columna con la desviación típica de los actuadores
%   mDest es una matriz 3xn con las poses de destino
%
%   Se permite que el primer parámetro sea una matriz para usar la función
%   en el filtro de partículas, donde cada partícula tiene una posición
%   inicial distinta y todas se actualizan con la misma orden
%
%   Pruebas
%       comp_nosiy([0 0 0]', [1 1 pi/4]', [0.1 0.01 0.001]')
%       comp_nosiy([0 0 0; 10 10 0]', [1 1 pi/4]', [0.1 0.01 0.001]')


    n=size(mOrig, 2); % número de partículas a actualizar
    mRuido = randn(3, n) .* cSigma; % matriz de ruido gaussiano
    mDest = zeros(3, n);
    for i=1:n
        % opcion a (clase) > componer el ruido
        uNoisy = comp_odom(cUpdate, mRuido(:,i)); % acción ruidosa
        mDest(:,i) = comp_odom(mOrig(:,i),uNoisy);

        % opcion b > sumar el ruido directamente
        %uNoisy = cUpdate + mRuido(:,i) % acción ruidosa        
        %mDest(:,i) = comp_odom(mOrig(:,i),uNoisy)
    end
end

