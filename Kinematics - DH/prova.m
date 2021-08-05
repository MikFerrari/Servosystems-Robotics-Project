clc; clear; close all;
%% caso 1
q=[0 0 0];
%Define DH parameters
D=[10 2 0]; A=[5 0 3]; T=[0 pi/2 -pi/2]; F=[+pi/2 -pi/2 -pi/2];
P03=[0;0;0;1];

%DH matrix
DHpar=[q(1)+T(1)  D(1)       A(1)   F(1);
       pi/2+T(2)  q(2)+D(2)  A(2)   F(2);
       q(3)+T(3)  D(3)       A(3)   F(3)];

M03=DHdirect(DHpar); %Solve direct kinematics

P0e=M03*P03; %End-effector position in S0
% P0e(:,4)=[];

% Plot WS
figure
plot3(P0e(1),P0e(2),P0e(3),'*g');
grid on
xlabel('x');ylabel('y');zlabel('z');
hold on
% P=[A(1) (D(2)+A(3))*cos(alpha) (D(2)+A(3))*sin(alpha)];
% plot3(P(1),P(2),P(3),'*r')

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