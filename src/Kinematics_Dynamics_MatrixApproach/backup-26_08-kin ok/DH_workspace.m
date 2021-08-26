% Joints variables 
nPoints=[30 15 15];
limits=pos_limits;
q1_inf = limits(1); q1_sup = limits(2); q1=linspace(q1_inf,q1_sup,nPoints(1));
q2_inf = limits(3); q2_sup = limits(4); q2=linspace(q2_inf,q2_sup,nPoints(2));
q3_inf = limits(5); q3_sup = limits(6); q3=linspace(q3_inf,q3_sup,nPoints(3));

Q = allCombinations(q1,q2,q3); % all possible combinations of joints variables
tic
for i=1:size(Q,1)
    P(i,:)=DHdirect(Q(i,1),Q(i,2),Q(i,3))';
end
toc

% Plot WS
figure
plot3(P(:,1),P(:,2),P(:,3),'.b');
xlabel('x');ylabel('y');zlabel('z');
title('Robot Workspace')
axis equal; grid on; view(45,20);
p.LineWidth = 3;
% set(gcf,'Visible','on')
