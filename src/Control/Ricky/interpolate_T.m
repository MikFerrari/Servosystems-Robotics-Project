
T1=actuatorTorques(2,:);
T2=actuatorTorques(3,:);
T3=actuatorTorques(4,:);

dT_ini = dT;
dT_target = 0.001;
nPoints_target = floor(dT*nPoints/dT_target);

tt_fine = linspace(tt(1),tt(end),nPoints_target);



T1_interp = interp1(tt,T1,tt_fine);
T2_interp = interp1(tt,T2,tt_fine);
T3_interp = interp1(tt,T3,tt_fine);

T_interp = [T1_interp; T2_interp; T3_interp];
