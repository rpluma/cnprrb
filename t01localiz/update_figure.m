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
            plot(Mapa.Landmarks(1,j),Mapa.Landmarks(2,j),'sg','LineWidth',4);
        else
            plot(Mapa.Landmarks(1,j),Mapa.Landmarks(2,j),'sr','LineWidth',4);
        end        
    end

    plot(Robot.xOdom(1),Robot.xOdom(2),'b+','LineWidth',2);   % Odométrica
    if (sum(bVisible)>=3 || bFP)
        plot(Robot.xEst(1) ,Robot.xEst(2), 'go','LineWidth',2);   % LSE o FP
    else
        plot(Robot.xEst(1) ,Robot.xEst(2), 'ro');   % LSE y <3 visibles
    end
    
    if bFP
        part=plot(Robot.fpPos(1, :), Robot.fpPos(2, :), '.');
        set(part, 'MarkerSize', 5, 'Color', [rand rand rand]);
    end
    plot(Robot.xTrue(1),Robot.xTrue(2),'kx','LineWidth',2);   % Real
    
end

