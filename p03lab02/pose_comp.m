function p=pose_comp(p1, p2)
    % This function computes the composition of two 3x1 poses p1,p2
    % p is the resultant pose
    
    % obtenemos theta1, su seno y su coseno
    t1=p1(3); 
    ct1=cos(t1); 
    st1=sin(t1);

    % definimos una matriz de rotación R para hacer la composición
    R=[ct1 st1 0; -st1 ct1 0; 0 0 1];

    % calculamos la pose como p1 + (p2'*R)'
    p = p1 + (p2' * R)';

    % restringimos el ángulo al intervalo [-pi, pi]
    p(3) = AngleWrap(p(3));
end