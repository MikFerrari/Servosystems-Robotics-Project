r = Q3_interp';
t = (1:length(Q3_interp))*0.001;

Pparams = load('system_params');
Pparams = Pparams.system_params;

popSize = 2;
maxGenerations = 1;

upBound = [1000 1 0 1000 1 0];
loBound = [0.1 0.1 0 0.1 0.1 0];

opt = optimoptions(@ga, 'PopulationSize',popSize,'MaxGenerations',maxGenerations,'MaxTime',10,'Display','iter');
warning off
[x,fval] = ga(@(Cparams)test_controller(Cparams,Pparams,r,t),6,[],[],[],[],loBound,upBound)
warning on


function J = test_controller(Cparams,Pparams,r,t)
    
    KpPos = Cparams(1);   TiPos = Cparams(2);   TdPos = Cparams(3);
    KpVel = Cparams(4);   TiVel = Cparams(5);   TdVel = Cparams(6);

    s = tf('s');
    
    P_int = Pparams.K_int*1/((Pparams.T1_int*s+1)*(Pparams.T2_int*s+1));
    P_ext = Pparams.K_ext*1/((Pparams.T1_ext*s+1)*(Pparams.T2_ext*s+1));


    C_int = KpVel*(1+1/(TiVel*s)+TdVel*s);
    C_ext = KpPos*(1+1/(TiPos*s)+TdPos*s);


    Fint = C_int*P_int/(1+C_int*P_int);
    Ftot = P_ext*C_ext*Fint/(1+P_ext*C_ext*Fint);

    y = real(lsim(Ftot,r,t));
    
    error = r-y;
    
    % Define cost function to minimize: IAE cost function
    J = sum(abs(error));

end