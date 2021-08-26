clc; clear; close all;
%%

DHdirect = M03*P3e;
DHdirect_MLF = matlabFunction(DHdirect,'Vars',[q1 q2 q3]);

%%
%     S = DHdirect(Q(1),Q(2),Q(3));
    
%     x0 = 0;                                         y0 = 0;                                         z0 = 0;
%     x1a = x0;                                       y1a = y0;                                       z1a = z0+D(1);
%     x1b = x0+A(1)*cos(Q(1));                        y1b = y0-A(1)*sin(Q(1));                        z1b = z1a;
%     x2 = x1b-(D(2)+Q(2))*cos(alpha)*sin(Q(1));      y2 = y1b-(D(2)+Q(2))*cos(alpha)*cos(Q(1));      z2 = z1b+(D(2)+Q(2))*sin(alpha);
%     x3 = x2-A(3)*cos(alpha)*sin(Q(1));              y3 = y2-A(3)*cos(alpha)*cos(Q(1));              z3 = z2+A(3)*sin(alpha+Q(3));
    
    
    x0 = 0;
    y0 = 0;
    z0 = 0;

    x1a = x0;
    y1a = y0;
    z1a = z0+D(1);

    x1b = x1a+A(1)*cos(Q(1));
    y1b = x1a+A(1)*sin(Q(1));
    z1b = z1a;

    x2 = x1b+(D(2)+Q(2))*cos(angle).*sin(Q(1));
    y2 = y1b-(D(2)+Q(2))*cos(angle).*cos(Q(1));
    z2 = z1b+(D(2)+Q(2))*sin(angle);

    x3 = x2+A(3)*cos(angle+Q(3)).*sin(Q(1));
    y3 = y2-A(3)*cos(angle+Q(3)).*cos(Q(1));
    z3 = z2+A(3)*sin(angle+Q(3));



































%% caso 1
q=[0 0 0];
%Define DH parameters
alpha=pi/6;
D=[2 4 0]; A=[1.1 0 1.5]; T=[0 pi/2 -pi/2]; F=[+pi/2 -pi/2 -pi/2];
P03=[0;0;0;1];

%DH matrix
DHpar=[q(1)+T(1)  D(1)       A(1)   F(1)-alpha;
       pi/2+T(2)  q(2)+D(2)  A(2)   F(2);
       q(3)+T(3)  D(3)       A(3)   F(3)];

M03=DHdirect(DHpar); %Solve direct kinematics

P0e=M03*P03; %End-effector position in S0
% P0e(:,4)=[];

% Plot WS
figure
plot3(P0e(1),P0e(2),P0e(3),'*b');
grid on
xlabel('x');ylabel('y');zlabel('z');
hold on
P=[A(1) -(D(2)+A(3))*cos(alpha) (D(2)+A(3))*sin(alpha)+D(1)];
plot3(P(1),P(2),P(3),'*r')

%% caso 2
L3=3;
D=[10 2 0]; A=[5 0 0]; T=[0 pi/2 0]; F=[+pi/2 -pi/2 pi/2];
P03=[0;0;0;L3];

%DH matrix
DHpar=[q(1)+T(1)  D(1)       A(1)   F(1);
       pi/2+T(2)  q(2)+D(2)  A(2)   F(2);
       q(3)+T(3)  D(3)       A(3)   F(3)];

M03=DHdirect(DHpar); %Solve direct kinematics

P0e=M03*P03; %End-effector position in S0
% P0e(:,4)=[];

% Plot WS
plot3(P0e(1),P0e(2),P0e(3),'*r');
grid on
xlabel('x');ylabel('y');zlabel('z');

%% Prova inv kin
alpha=pi/6;
D=[2 4 0]; A=[1.1 0 1.5]; T=[0 pi/2 -pi/2]; F=[+pi/2 -pi/2 -pi/2];
P0e=[1.10000000000000;-4.76313972081441;4.75000000000000];
[q1,q2,q3] = DHinv(P0e,D,A,alpha)
%%
tic
for i=1:size(Q,1)
    P(:,i)=double(P0e(Q(i,1),Q(i,2),Q(i,3)));
end
toc

