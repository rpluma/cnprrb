function [xProb] = mover_particulas(Robot, bVisualizar)
%MOVER_PARTICULAS Mueve la posición de cada partícula con un ruido 
% aleatorio distinto
%   Robot.uOdom contiene la acción que queremos hacer (avanzar o girar)
%   bVisualizar es un booleano para graficar o no las partículas

    % inicialización de la nueva posición de las partículas
    xProb=zeros(3, Robot.Particles);

    % generación de partículas
    for j=1:Robot.Particles
        % ruido aleatorio
        noise = Robot.uStd .* randn(3, 1);

        % movimiento con ruido aleatorio
        unoisy = pose_comp(Robot.uOdom, noise);

        % nueva posición de la partícula
        xProb(:,j)=pose_comp(Robot.xProb(:,j), unoisy);
    end

    % Visualizar si es preciso
    if (bVisualizar)
        part=plot(xProb(1,:), xProb(2,:), '.');
        set(part, 'MarkerSize', 5, 'Color', [rand rand rand]);
    end
end

