function intersects = Intersects(p1, p2, p3, p4)
%intersects = true se il segmento p1p2 interseca il segmento p3p4, 
%false altrimenti
intersects = false;
delta0 = (p4(2)-p3(2))*(p2(1)-p1(1))-(p4(1)-p3(1))*(p2(2)-p1(2));
if(delta0~=0) %se delta0=0 i segmenti sono paralleli
    %se i numeratori delta1 e delta2 sono nulli, allora i segmenti sono
    %coincidenti
    delta1 = (p4(1)-p3(1))*(p1(2)-p3(2))-(p4(2)-p3(2))*(p1(1)-p3(1));
    delta2 = (p2(1)-p1(1))*(p1(2)-p3(2))-(p2(2)-p1(2))*(p1(1)-p3(1));
    ka = delta1/delta0;
    kb = delta2/delta0;
    %se 0<ka<1 strettamente allora il punto di intersezione cade entro p1p2
    %se 0<kb<1 strettamente allora il punto di intersezione cade anche
    %entro p3p4
    if((ka>=0 && ka<=1) && (kb>=0 && kb<=1))
        intersects = true;
    end
end
end