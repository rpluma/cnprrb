function Landmarks=crear_landmarks(Mapa)
    
    % Landmarks uniformly distributed in map
    Landmarks = Mapa.Size*rand(2, Mapa.nLandmarks)-Mapa.Size/2; 

    % preparar la visualización
    close all;
    figure(1);
    set(gcf,'Visible','on');    % pup-up window
    hold on; 
    grid on; 
    axis equal;

    % plot landmarks (magenta squares)
    plot(Landmarks(1,:),Landmarks(2,:),'sm','LineWidth',4);

    % Plot Labels
    for k = 1:Mapa.nLandmarks
        text( Landmarks(1,k)+1,Landmarks(2,k), sprintf('L%d',k) );
    end
end
