Ttot=tt(end);

if task == 1
    
    KpPos_1 = 1;        KpVel_1 = 1000;
    TiPos_1 = 0.1;      TiVel_1 = 0.5;
    TdPos_1 = 0;        TdVel_1 = 0;
    
    KpPos_2 = 100;      KpVel_2 = 10;
    TiPos_2 = 0.01;     TiVel_2 = 10;
    TdPos_2 = 0;        TdVel_2 = 1;

    KpPos_3 = 1;        KpVel_3 = 1000;
    TiPos_3 = 0.1;      TiVel_3 = 100;
    TdPos_3 = 0;        TdVel_3 = 0;
    
elseif task == 2
    
    KpPos_1 = 1;    KpVel_1 = 1;
    TiPos_1 = 50;  TiVel_1 = 100;
    TdPos_1 = 0;    TdVel_1 = 0;
    
    KpPos_2 = 1;    KpVel_2 = 1;
    TiPos_2 = 100;  TiVel_2 = 100;
    TdPos_2 = 0;     TdVel_2 = 0;

    KpPos_3 = 1;    KpVel_3 = 1;
    TiPos_3 = 100;  TiVel_3 = 100;
    TdPos_3 = 0;    TdVel_3 = 0;
    
elseif task == 3
    
    KpPos_1 = 10;    KpVel_1 = 1000;
    TiPos_1 = 1;  TiVel_1 = 0.1;
    TdPos_1 = 0;    TdVel_1 = 0;
    
    KpPos_2 = 500;    KpVel_2 = 10;
    TiPos_2 = 0.1;  TiVel_2 = 1;
    TdPos_2 = 0;     TdVel_2 = 1;

    KpPos_3 = 10;    KpVel_3 = 100;
    TiPos_3 = 0.1;    TiVel_3 = 0.01;
    TdPos_3 = 0;    TdVel_3 = 0;
    
end