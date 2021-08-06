clc; clear; close all;
caso=1;

%Define DH parameters
if caso==1
    D=[5 0 0]; A=[2 0 1]; T=[0 pi/2 -pi/2]; F=[+pi/2 -pi/2 -pi/2];
    P03=[0;0;0;1];
elseif caso==2
    L3=3;
    D=[5 0 0]; A=[2 0 0]; T=[0 pi/2 0]; F=[+pi/2 -pi/2 pi/2];
    P03=[0;0;0;L3];
end

%Joints variables 
nPoints=25;
limits=[0 2*pi 0 3 -pi*0.5 pi*0.5];
q1_inf = limits(1); q1_sup = limits(2);
q2_inf = limits(3); q2_sup = limits(4);
q3_inf = limits(5); q3_sup = limits(6);

%All possible combinations in joints space
q1=linspace(q1_inf,q1_sup,nPoints); %q1 → theta1
q2=linspace(q2_inf,q2_sup,nPoints); %q2 → d2
q3=linspace(q3_inf,q3_sup,nPoints); %q3 → theta3

%Combinazioni possibili delle variabili ai giunti
elements = {q1,q2,q3}; %cell array with N vectors to combine
combinations = cell(1, numel(elements)); %set up the varargout result
[combinations{:}] = ndgrid(elements{:});
combinations = cellfun(@(x) x(:), combinations,'uniformoutput',false); %there may be a better way to do this
Q = [combinations{:}]; % NumberOfCombinations by N matrix. Each row is unique.

%% Rappresentazione WS come insieme di punti
for i=1:size(Q,1)
    q=Q(i,:);
    alpha=pi/6;
    
    %DH matrix
    DHpar=[q(1)+T(1)  D(1)       A(1)   F(1)-alpha;
           pi/2+T(2)  q(2)+D(2)  A(2)   F(2);
           q(3)+T(3)  D(3)       A(3)   F(3)];
    
    M03=DHdirect(DHpar); %Solve direct kinematics
    
    P0e(i,:)=M03*P03; %End-effector position in S0
end
% P0e(:,4)=[];

% Plot WS
figure
plot3(P0e(:,1),P0e(:,2),P0e(:,3),'.-b');
grid on
xlabel('x');ylabel('y');zlabel('z');
xlim([-10 10]);ylim([-10 10]);zlim([0 10]);
% axis equal
% view(30,30)