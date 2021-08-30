


    
Q = 1;
R = 0.001;
popSize = 2;
maxGenerations = 1;
initPop = ones(popSize,6);

upBound = [1000 1 0 1000 1 0];
loBound = [0.1 0.1 0 0.1 0.1 0];

opt = optimoptions(@ga, 'PopulationSize',popSize,'MaxGenerations',maxGenerations, ...
                   'InitialPopulationMatrix',initPop,'MaxTime',120);
warning off
[x,fval] = ga(@(K)test_controller(K,Q,R),6,[],[],[],[],loBound,upBound);
warning on



function J = test_controller(parms,Q,R)
    
    assignin('base','KpPos_3',parms(1));
    assignin('base','TiPos_3',parms(2));
    assignin('base','TdPos_3',parms(3));
    assignin('base','KpVel_3',parms(4));
    assignin('base','TiVel_3',parms(5));
    assignin('base','TdVel_3',parms(6));

    paramNameValStruct.SimMechanicsOpenEditorOnUpdate = 'off';
    sim('manipulator_identification_q3',paramNameValStruct);

    y_SP = evalin('base','out.q3_SP(:).Data');
    y_PV = evalin('base','out.y(:).Data');
    MV = evalin('base','out.u(:).Data');
    
    % Define cost function to minimize: LQR cost function
    J = sum(Q*(y_SP-y_PV).^2+R*MV.^2);

end