function a = AngleWrap(a)
    % añadimos 11 vueltas y media para que el ángulo sea positivo
    % después restamos el número entero de vueltas completas
    % y finalmente restamos la media vuelta para que el ángulo 
    % resultante quede entre -pi y +pi
    a = a + (10*2+1)*pi; % añadimos 10 vueltas y media para asegurar a>0
    vueltas = floor(a/(2*pi)); % calculamos el número entero de vueltas
    a = a - vueltas * (2*pi) - pi; % quitamos vueltas y restmoas 180º
end
