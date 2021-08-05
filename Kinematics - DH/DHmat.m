function M=DHmat(v)
%v: vettore contente i parametri di DH per un link
    M=[cos(v(1)) -sin(v(1))*cos(v(4)) sin(v(1))*sin(v(4)) v(3)*cos(v(1));
       sin(v(1)) cos(v(1))*cos(v(4)) -cos(v(1))*sin(v(4)) v(3)*sin(v(1));
       0         sin(v(4))            cos(v(4))           v(2);
       0         0                    0                   1];