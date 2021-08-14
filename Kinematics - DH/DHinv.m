function [Q,q1,q2,q3] = DHinv(P,D,A,alpha)

    x=P(1); y=P(2); z=P(3);
    Q=[];

    % q1
    if x>0 
        phi = atan(x/y);
    elseif x<0 
        phi = atan(x/y)+pi;
    end
    
    R=(x^2+y^2)^0.5; 
    q1 = [asin(A(1)/R)-phi-pi; pi-asin(A(1)/R)-phi-pi];
    
    % q3
    for i=1:size(q1,1)
        
        q3(i,1) = asin( ((y-A(1)*sin(q1(i)))*sin(alpha)/cos(q1(i))+(z-D(1))*cos(alpha))/A(3) );
        q3(i,2) = - pi - asin( ((y-A(1)*sin(q1(i)))*sin(alpha)/cos(q1(i))+(z-D(1))*cos(alpha))/A(3) );
        
        % q2
        for j=1:size(q3(i,:),2)
            q2(i,j)=( z-D(1)-A(3)*sin(alpha+q3(i,j)) )/sin(alpha) - D(2);
            Q=[Q; q1(i) q2(i,j) q3(i,j)];
        end
    end
    
    
    
    %     q1(1,1)=asin(A(1)/(x^2+y^2)^0.5)-atan2(x,y);
    %     q1(2,1)=pi-asin(A(1)/(x^2+y^2)^0.5)-atan2(x,y);

    %     q1(1,1)=acos(A(1)/(x^2+y^2)^0.5)+atan(y/x);
    %     q1(2,1)=-acos(A(1)/(x^2+y^2)^0.5)+atan(y/x);
    



    