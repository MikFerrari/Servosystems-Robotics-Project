clc
clear all
close all

%Define robot parameters
d1=10;
a1=1;
d3=5;

%Define DH parameters
nPoints=25;
q1min=0; q1max=pi; q1=linspace(q1min,q1max,nPoints); %q1 → theta1
q2min=0; q2max=10; q2=linspace(q2min,q2max,nPoints); %q2 → d2
q3min=0; q3max=pi; q3=linspace(q3min,q3max,nPoints); %q3 → theta3

%All possible combinations - WS points
elements = {q1,q2,q3}; %cell array with N vectors to combine
combinations = cell(1, numel(elements)); %set up the varargout result
[combinations{:}] = ndgrid(elements{:});
combinations = cellfun(@(x) x(:), combinations,'uniformoutput',false); %there may be a better way to do this
Q = [combinations{:}]; % NumberOfCombinations by N matrix. Each row is unique.

%% Rappresentazione WS come insieme di punti
for i=1:size(Q,1)
    q=Q(i,:);
    qa=q(1);
    qb=q(2);
    qc=q(3);
    
    %DH table
    N_link=linspace(1,3,3)';
    J_type=["R","P","R"]';
    theta=[qa;0;qc];
    d=[d1;qb;0];
    a=[a1;0;0];
    phi=[0;-pi/2;pi/2];
    DH_table=table(N_link,J_type,theta,d,a,phi);

    %DH Matrices
    M01=[cos(qa) -sin(qa) 0 a1*cos(qa);
         sin(qa) cos(qa)  0 a1*sin(qa);
         0       0        1 d1;
         0       0        0 1];

    M12=[1  0 0  0;
         0  0 1  0;
         0 -1 0  qb;
         0  0 0  1];

    M23=[cos(qc) 0   sin(qc) 0;
         sin(qc) 0  -cos(qc) 0;
         0       1   0       0;
         0       0   0       1];

     M03=M01*M12*M23;

     %End-effector position in S0
     P3e=[0;0;d3;1];
     P0e(i,:)=M03*P3e;
end

% Plot WS
figure
plot3(P0e(:,1),P0e(:,2),P0e(:,3));

%% Rappresentazione WS come funzione
f = @(x,y,z) x.^2+y.^2+z.^2-3*x.*y.*z ;
x = linspace(-1,1) ;
y = linspace(-1,1) ;
z = linspace(-1,1) ;
[X,Y,Z] = ndgrid(x,y,z) ;
F =f(X,Y,Z) ;
figure
hold on
for i = 1:100
    surf(X(:,:,i),Y(:,:,i),Z(:,:,i),F(:,:,i)) ;
end

f=@(x,y,z) (x.^2+y.^2+z.^2)^0.5;
X=P0e(:,1);
Y=P0e(:,2);
Z=P0e(:,3);
F=f(X,Y,Z);

figure
hold on
for i = 1:nPoints
    surf(X(:),Y(:),Z(:)) ;
end

%%
syms X Y Z
f = @(x,y,z) x^2 + y^2 + z^2;
surff = f(X,Y,Z);
fimplicit3(surff)

%%
plot3(X,Y,Z)