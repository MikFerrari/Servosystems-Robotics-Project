function M03=DHdirect(DHpar)
% Dpar: matrice con i parametri di DH
%Gli passo una tabella completa (per ogni punto) e calcolo la M03.
%Mi serve una funzione che calcoli una matrice di DH dati i valori di una riga.

    M01=DHmat(DHpar(1,:));
    M12=DHmat(DHpar(2,:));
    M23=DHmat(DHpar(3,:));

    M03=M01*M12*M23;