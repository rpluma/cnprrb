function [Mapa, fg] = create_map(Size, nLandmarks)
%CREATE_MAP Crea un mapa con sus Landmarks e inicializa la visualización
%   Size es el tamaño del mapa donde se ubicarán los landmarks (m)
%   nLandmarks es el número de Landmarks aleatorios a crear
%   iFig es el número de la figura a crear
%   Prueba: 
%       Size=5; nLandmarks=10, iFig=2; Mapa=create_map(Size, nLandmarks, 3);

    % inicializar el mapa
    Mapa={};
    Mapa.nLandmarks = nLandmarks;
    Mapa.Size = Size;
    Mapa.Landmarks = Size*rand(2, nLandmarks); 
    Mapa.Landmarks = Mapa.Landmarks - [Size/2 Size/2]';

    % preparar la visualización
    fg=figure();
    set(gcf,'Visible','on');    % pup-up window
    
    % visualizar las esquinas dejando margen alrededor
    mg = Size*0.75;
    plot([-mg -mg mg mg], [-mg mg -mg mg], 'k+');
    hold on; grid on; axis equal;
    
    % plot landmarks (magenta squares)
    plot(Mapa.Landmarks(1,:),Mapa.Landmarks(2,:),'sm','LineWidth',4);

    % Plot Labels
    for k = 1:Mapa.nLandmarks
        text(Mapa.Landmarks(1,k)+1,Mapa.Landmarks(2,k), sprintf('L%d',k) );
    end

end

