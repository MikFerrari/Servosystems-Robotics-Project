function [q1,q2,q3,q1p,q2p,q3p,q1pp,q2pp,q3pp] = trajectory_gen(t,dT)
    
    if abs(t-dT) <= 1e-2
        
        i = floor(t/dT)+1;
        q1 = Q(1,i);
        q2 = Q(2,i);
        q3 = Q(3,i);
        q1p = Qp(1,i);
        q2p = Qp(2,i);
        q3p = Qp(3,i);
        q1pp = Qpp(1,i);
        q2pp = Qpp(2,i);
        q3pp = Qpp(3,i);
        
    end
    
        


