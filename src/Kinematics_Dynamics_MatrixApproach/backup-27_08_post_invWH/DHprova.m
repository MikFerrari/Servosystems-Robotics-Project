clear S prova
S = SS(:,4,:); S = reshape(S,3,100);
Sp = SSp(:,4,:); Sp = reshape(Sp,3,100);
Spp = SSpp(:,4,:); Spp = reshape(Spp,3,100);

Qp;
Q;

vel = Qpinv(Q(1,:),Q(2,:),Q(3,:),Sp(1,:),Sp(2,:),Sp(3,:));
acc = Qppinv(Q(1,:),Q(2,:),Q(3,:),Qp(1,:),Qp(2,:),Qp(3,:),Sp(1,:),Sp(2,:),Sp(3,:),Spp(1,:),Spp(2,:),Spp(3,:));
