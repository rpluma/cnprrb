function [] = update_figure(Robot, Mapa, bVisible, bFP)
%UPDATE_FIGURE Actualiza el mapa (color de landmarks) y el robot
%   Posición real con una x en negro
%   Posición odométrica con una línea en rojo
%   Posición estimada con color dependiente de tipo de estimación
%   Nube de puntos dependiente de si bFP = true
%   prueba
%       update_figure(Robot, Mapa, bVisible, 0);

    % visualiza el color de los landmarks en función de si están visibles
    for j=1:Mapa.nLandmarks
        if bVisible(j)
            cl = 'sg'; ce='go'; % landmark verde y estimación verde
        else
            cl = 'sr'; ce='ro'; % landmar roja y estimación roja
        end
        plot(Mapa.Landmarks(1,j),Mapa.Landmarks(2,j),cl,'LineWidth',4);
    end

    plot(Robot.xTrue(1),Robot.xTrue(2),'kx');   % Real
    plot(Robot.xOdom(1),Robot.xOdom(2),'b+');   % Odométrica
    plot(Robot.xEst(1) ,Robot.xEst(2), ce);   % Estimada
    if bFP
        part=plot(Robot.fpPos(1, :), fpPos(2, :), '.');
        set(part, 'MarkerSize', 5, 'Color', [rand rand rand]);
    end
end

