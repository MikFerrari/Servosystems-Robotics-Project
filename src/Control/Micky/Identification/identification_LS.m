%% Open Simulink scheme 'manipulator_identification_q3'
% and uncomment the blocks related to the process you want to identify
% (2nd order t.f. approximation)


%% Simulate the system 5 times using the following controller parameters:

YY_int = [];
PHIPHI_int = [];

YY_ext = [];
PHIPHI_ext = [];
    
paramNameValStruct.SimMechanicsOpenEditorOnUpdate = 'off';
for i = 1:5
    
    switch i
        case 1
            KpPos_3 = 1;      KpVel_3 = 1000;
            TiPos_3 = 0.1;    TiVel_3 = 100;
            TdPos_3 = 0;      TdVel_3 = 0;
        case 2
            KpPos_3 = 1;      KpVel_3 = 1000;
            TiPos_3 = 0.1;    TiVel_3 = 10;
            TdPos_3 = 0;      TdVel_3 = 0;
        case 3
            KpPos_3 = 1;      KpVel_3 = 100;
            TiPos_3 = 0.1;    TiVel_3 = 10;
            TdPos_3 = 0;      TdVel_3 = 0;
        case 4
            KpPos_3 = 1;      KpVel_3 = 100;
            TiPos_3 = 0.1;    TiVel_3 = 1;
            TdPos_3 = 0;      TdVel_3 = 0;
        case 5
            KpPos_3 = 1;      KpVel_3 = 10;
            TiPos_3 = 0.1;    TiVel_3 = 1;
            TdPos_3 = 0;      TdVel_3 = 0;
    end

    sim('manipulator_identification_q3',paramNameValStruct);
    
    Y_int = out.y_int.Data;
    PHI_int = [out.int_1_y_int.Data out.int_2_y_int.Data out.int_2_u.Data];

%     save(strcat(Y_int,'_',i),Y_int)
%     save(strcat(PHI_int,'_',i),PHI_int)

    Y_ext = out.y.Data;
    PHI_ext = [out.int_1_y.Data out.int_2_y.Data out.int_2_y_int.Data];

%     save(strcat(Y_ext,'_',i),Y_ext)
%     save(strcat(PHI_ext,'_',i),PHI_ext)
    
    YY_int = [YY_int; Y_int];
    PHIPHI_int = [PHIPHI_int; PHI_int];
    
    YY_ext = [YY_ext; Y_ext];
    PHIPHI_ext = [PHIPHI_ext; PHI_ext];
    
end

theta_int = (PHIPHI_int'*PHIPHI_int)\(PHIPHI_int'*YY_int);
theta_ext = (PHIPHI_ext'*PHIPHI_ext)\(PHIPHI_ext'*YY_ext);

K_int = -theta_int(3)/theta_int(2);

T2_int = (theta_int(1)/2-sqrt(theta_int(1)^2+4*theta_int(2)))/(2*theta_int(2));
T1_int = theta_int(1)/theta_int(2)-T2_int;

K_ext = -theta_ext(3)/theta_ext(2);

T2_ext = (theta_ext(1)/2-sqrt(theta_ext(1)^2+4*theta_ext(2)))/(2*theta_ext(2));
T1_ext = theta_ext(1)/theta_ext(2)-T2_ext;

clear system_params

system_params.K_int = K_int;
system_params.T1_int = T1_int;
system_params.T2_int = T2_int;
system_params.K_ext = K_ext;
system_params.T1_ext = T1_ext;
system_params.T2_ext = T2_ext;

save system_params system_params