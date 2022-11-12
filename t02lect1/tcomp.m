function [p] = tcomp(p1, p2)
    % ----------------- 1 Extraer los componentes de cada pose
    x1 = p1[1];
    y1 = p1[2];
    t1 = p1[3];

    x2 = p2[1];
    y2 = p2[2];
    t2 = p1[3];

    % ----------------- 2 Componer los ángulos y ajustar a [-Pi,Pi]
    t3 = t1 + t2;
    if (t3 > pi)
        t3 = t3 - 2*pi;
    elseif(t3 < -pi)
        t3 = t3 + 2*pi;
    end

    % ---------------- 3 Componern las posiciones
    c = cos(t1);
    s = sin(t1);
    p = [x1 + x2*c - y2*s, y1 + x2*s + y2*c, t3]
end