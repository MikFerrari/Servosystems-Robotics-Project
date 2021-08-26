%% Compute derivatives (numerical differentiation) to debug results

% DERIVATIVE OF TOTAL ENERGY and JUST KINETIC ENERGY
% E_tot_diff = diff(E_tot)/dT;
% Ek_tot_diff = diff(Ek_tot)/dT;

% VELOCITY AND ACCELERATION - GRIPPER COORDINATES
SSp_diff = zeros(3,nLinks,nPoints-1);
SSpp_diff = zeros(3,nLinks,nPoints-1);
VV_diff = zeros(1,nLinks,nPoints-1);
AA_diff = zeros(1,nLinks,nPoints-2);

dT = Ttot/nPoints;

for j = 2:nLinks
    S = reshape(SS(:,j,:),3,[]);
    SSp_diff(:,j,:) = diff(S,[],2)/dT;
    
    Sp = reshape(SSp(:,j,:),3,[]);
    SSpp_diff(:,j,:) = diff(Sp,[],2)/dT;
    
%     for i = 2:nPoints-1
%         if i == 1
%             VV_diff(:,j,i) = norm(SSp(:,j,i));
%             AA_diff(:,j,i) = norm(SSpp(:,j,i));
%         elseif i > 1
%             if PP(:,j,i) >= PP(:,j,i-1)
%                 VV_diff(:,j,i) = norm(SSp(:,j,i));      end
%             if PP(:,j,i) < PP(:,j,i-1)
%                 VV_diff(:,j,i) = -norm(SSp(:,j,i));     end
%             if VV_diff(:,j,i) >= VV_diff(:,j,i-1)
%                 AA_diff(:,j,i) = norm(SSpp(:,j,i));     end
%             if VV_diff(:,j,i) < VV_diff(:,j,i-1)
%                 AA_diff(:,j,i) = -norm(SSpp(:,j,i));    end
%         end
%     end

    VV_diff(:,j,:) = diff(PP(:,j,:))/dT;
    AA_diff(:,j,:) = diff(VV_diff(:,j,:))/dT;
end

% VELOCITY AND ACCELERATION - JOINT COORDINATES
Qp_diff = diff(Q,[],2)/dT;
Qpp_diff = diff(Qp,[],2)/dT;