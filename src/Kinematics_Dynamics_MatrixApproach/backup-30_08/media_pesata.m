function q = media_pesata(t,Q,dT,j)
        % Q vettore colonna
        
        i = floor(t/dT);
        qa = Q(j,i-1);
        qb = Q(j,i);
        qc = Q(j,i+1);
        
        q = (qa*dT*(i-1) + qb*dT + qc*dT*(i+1))/(dT*(i-1)+dT+dT*(i+1));