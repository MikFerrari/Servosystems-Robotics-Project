%%%%% LOOP FOR EACH TIME STEP %%%%%

if task ~= 2
    tt = linspace(0,Ttot,nPoints);
elseif task == 2
    tt = (0:(nPoints-1))*dT;
    Q0 = num2cell(Q_initial_shape);
end


for i = 1:nPoints
    
    %%%%% KINEMATICS (from the base link to the end-effector) %%%%%
    
    % JOINT COORDINATES
    % Cycloidal law of motion
    if task == 1
        [q1,q1p,q1pp] = cycloidal(tt(i),Ttot,Q_home(1),dQ(1));
        [q2,q2p,q2pp] = cycloidal(tt(i),Ttot,Q_home(2),dQ(2));
        [q3,q3p,q3pp] = cycloidal(tt(i),Ttot,Q_home(3),dQ(3));
        
        % Save joint coordinates
        Q(:,i) = real([q1; q2; q3]);
        Qp(:,i) = real([q1p; q2p; q3p]);
        Qpp(:,i) = real([q1pp; q2pp; q3pp]);
        
    elseif task == 2
        
        % JOINT COORDINATES THROUGH INVERSE KINEMATICS
        [temp,~,~,~] = DHinv(S_shape(:,i),D,A,alpha);
        Q(:,i) = temp(1,:)';
        q1=Q(1,i); q2=Q(2,i); q3=Q(3,i);

        % VELOCITY ARRAY
        Qp(:,i) = Qpinv(Q(1,i),Q(2,i),Q(3,i),Sp_shape_diff(1,i),Sp_shape_diff(2,i),Sp_shape_diff(3,i));

        % ACCELERATION ARRAY      
        Qpp(:,i) = Qppinv(Q(1,i),Q(2,i),Q(3,i),...
                         Qp(1,i),Qp(2,i),Qp(3,i),...
                         Sp_shape_diff(1,i),Sp_shape_diff(2,i),Sp_shape_diff(3,i),...
                         Spp_shape_diff(1,i),Spp_shape_diff(2,i),Spp_shape_diff(3,i));
        
        
    elseif task == 3
        [q1,q1p,q1pp] = cycloidal(tt(i),Ttot,Q_final_shape(1),dQ(1));
        [q2,q2p,q2pp] = cycloidal(tt(i),Ttot,Q_final_shape(2),dQ(2));
        [q3,q3p,q3pp] = cycloidal(tt(i),Ttot,Q_final_shape(3),dQ(3));        
        
        % Save joint coordinates
        Q(:,i) = real([q1; q2; q3]);
        Qp(:,i) = real([q1p; q2p; q3p]);
        Qpp(:,i) = real([q1pp; q2pp; q3pp]);
    end

    % LOOP THROUGH EACH LINK TO COMPUTE POSITION, VELOCITY, ACCELERATION MATRICES
    
    % BASE LINK
    MM_0(:,:,1,i) = M00_0;
    WW_0(:,:,1,i) = W00_0;
    HH_0(:,:,1,i) = H00_0;
    MMg_prev(:,:,1,i) = Mg00_prev;
    
    
    for j = 2:nLinks
        
        % POSITION OF REFERENCE FRAME
        % Position matrix of the current frame wrt the PREVIOUS frame
        M_rel_prevRef = Mall{j-1}(q1,q2,q3);
        Mg_rel = Mall_G{j-1}(q1,q2,q3);
        
%         Mg_rel =  M_rel_prevRef + [zeros(3) Tg;
%                                    [0 0 0] 1];

        % Position matrix of the current frame wrt the BASE frame
        M_abs_baseRef = MM_0(:,:,j-1,i)*M_rel_prevRef;
        % Save new matrix into position matrices array
        MM_0(:,:,j,i) = M_abs_baseRef;
        
        % Position matrix of the CENTER OF MASS wrt the previous frame
%         Mg_rel_prevRef = [   R    Tg;
%                           [0 0 0]  1];

        % Save new matrix into position matrices array
        MMg_prev(:,:,j,i) = Mg_rel;
        
        % VELOCITY OF REFERENCE FRAME
        % Current frame wrt the PREVIOUS frame expressed in the PREVIOUS frame
        W_rel = LL_prev(:,:,j-1).*Qp(j-1,i);          
        % Current frame wrt the PREVIOUS frame expressed in the BASE frame
        W_rel_0 = MM_0(:,:,j-1,i)*W_rel*MM_0(:,:,j-1,i)^(-1);
        % Current frame wrt the BASE frame expressed in the BASE frame (RIVALS' THEOREM)
        W_abs_baseRef = WW_0(:,:,j-1,i)+W_rel_0;      
        % Save new matrix into velocity matrices array
        WW_0(:,:,j,i) = W_abs_baseRef;                            
        WW_rel_0(:,:,j,i) = W_rel_0;
                
        % ACCELERATION OF REFERENCE FRAME
        % Current frame wrt the PREVIOUS frame expressed in the PREVIOUS frame
        H_rel = LL_prev(:,:,j-1)*Qpp(j-1,i)+LL_prev(:,:,j-1)^2*Qp(j-1,i)^2;
        % Current frame wrt the PREVIOUS frame expressed in the BASE frame
        H_rel_0 = MM_0(:,:,j-1,i)*H_rel*MM_0(:,:,j-1,i)^(-1);
        % Current frame wrt the BASE frame expressed in the BASE frame (CORIOLIS' THEOREM)
        H_abs_baseRef = HH_0(:,:,j-1,i)+H_rel_0+2*WW_0(:,:,j-1,i)*W_rel_0;
        % Save new matrix into acceleration matrices array
        HH_0(:,:,j,i) = H_abs_baseRef;
        
        % JOINT ABSOLUTE XYZ POSITION, VELOCITY, ACCELERATION
        SS(:,j,i) = MM_0(1:end-1,4,j,i);
        SSp(:,j,i) = WW_0(1:end-1,:,j,i)*MM_0(:,4,j,i);
        SSpp(:,j,i) = HH_0(1:end-1,:,j,i)*MM_0(:,4,j,i);
        
        % JOINT ABSOLUTE POSITION, VELOCITY, ACCELERATION ALONG THE TRAJECTORY
        if i ~= 1
            PP(:,j,i) = PP(:,j,i-1) + norm(SS(:,j,i) - SS(:,j,i-1)); 
        elseif i == 1
            PP(:,j,i) = 0;
        end
    end
    
    
    %%%%% END KINEMATICS %%%%%
    
    %%{
    %%%%% DYNAMICS (from the end-effector to the base link) %%%%%      
    
    % CONVERT EXTERNAL FORCES-TORQUES ON THE GRIPPER TO THE BASE REFERENCE FRAME
    % External forces and torques are defined in an auxiliary frame
    M0a_0 = eye(4);
    M0a_0(1:3,4) = MM_0(1:3,4,4,i);
    Phi_ext_0 = M0a_0*Phi_ext_a*M0a_0';
    
    % VELOCITY MATRIX OF THE AUXILIARY FRAME WRT THE BASE FRAME
    W0a_0 = zeros(4);
    W0a_0(1:3,4) = SSp(:,4,i);
        
    for j = nLinks:(-1):1
        
        % CONVERT L MATRICES TO THE BASE REFERENCE FRAME
        if j > 1
            LL_0(:,:,j-1) = MM_0(:,:,j-1,i)*LL_prev(:,:,j-1)*MM_0(:,:,j-1,i)^(-1);
        end
        
        % CONVERT J MATRICES TO THE REFERENCE FRAME
        JJ_0(:,:,j,i) = MM_0(:,:,j,i)*JJ_end(:,:,j)*MM_0(:,:,j,i)';

        % BODY WEIGHT IN THE BASE REFERENCE FRAME (no need to convert)
        Phi_g = skew(Hg*JJ_0(:,:,j,i));
        Phi_g_array_0(:,:,j,i) = Phi_g;
        
        % INERTIAL FORCES AND TORQUES IN THE BASE REFERENCE FRAME (no need to convert)
        Phi_in = skew(HH_0(:,:,j,i)*JJ_0(:,:,j,i));
        Phi_in_array_0(:,:,j,i) = Phi_in;
        
        % VINCULAR REACTION TRANSMITTED FROM THE CURRENT LINK TO THE PREVIOUS ONE,
        % EXPRESSED IN THE BASE REFERENCE FRAME
        if j == nLinks
            Phi_vinc_array_0(:,:,j,i) = -Phi_in_array_0(:,:,j,i)+Phi_g_array_0(:,:,j,i)+Phi_ext_0;
        else
            Phi_vinc_array_0(:,:,j,i) = -Phi_in_array_0(:,:,j,i)+Phi_g_array_0(:,:,j,i)+Phi_vinc_array_0(:,:,j+1,i);
        end
        
        % ACTUATOR FORCES AND TORQUES IN THE JOINT BETWEEN THE CURRENT LINK AND THE PREVIOUS ONE
        if j > 1
            phi_actuators_array_0(1,1,j,i) = -pseudoScalar(Phi_vinc_array_0(:,:,j,i),LL_0(:,:,j-1));
        end
        
        % KINETIC & POTENTIAL ENERGY OF THE CURRENT LINK WRT THE BASE FRAME
        Ek(j,i) = trace(0.5*WW_0(:,:,j,i)*JJ_0(:,:,j,i)*WW_0(:,:,j,i)');
        Ep(j,i) = -trace(Hg*JJ_0(:,:,j,i));
        E(j,i) = Ek(j,i)+Ep(j,i);   % total mechanical energy
        
        % MOTOR POWER IN THE CURRENT JOINT (frame-independent?)
        if j > 1
            Wq(j,i) = -pseudoScalar(Phi_vinc_array_0(:,:,j,i),WW_rel_0(:,:,j,i));
            % "-" because "Phi_vinc_array_0" applied to the link is the
            % opposite to "Phi_vinc_array_0" transmitted to the previous one
            
            Wweight_link(j,i) = pseudoScalar(Phi_g_array_0(:,:,j,i),WW_0(:,:,j,i));
            % "+" because "Phi_g_array_0" is a force that acts in the
            % absolute frame and "WW_0" are absolute velocity matrices
            
%             Wweight_link(j,i) = trace(WW_0(:,:,j,i)*JJ_0(:,:,j,i)*Hg'); % Alternative method to compute power
        end
        
    end
    
    % EXTERNAL FORCES-TORQUES POWER (frame-independent?)
    Wext(i) = pseudoScalar(Phi_ext_0,W0a_0);  
    % "+" because "Phi_ext_0" is a force that acts in the
    % absolute frame and "W0a_0" is an absolute velocity matrix
            
    % TOTAL POWER (from motors and external forces/torques)
    W(i) = sum(Wq(:,i))+Wext(i);	
    
    % POWER OF THE WEIGHT FORCES
    Wweight(i) = sum(Wweight_link(:,i));
    
    %%%%% END DYNAMICS %%%%%
%}      
end 
  
%%%%% END LOOP FOR EACH TIME STEP %%%%%