%% Inverse W & H - method I.I
syms xp yp zp xpp ypp zpp q1 q2 q3 q1p q2p q3p q1pp q2pp q3pp wx wy wz

% Conversioni
M02sym = simplify(M01sym*M12sym);
L01_0sym = L01_0;
L12_0sym = simplify(M01sym*L12_1*inv(M01sym));
L23_0sym = simplify(M02sym*L23_2*inv(M02sym));

W03_0sym=simplify(L01_0sym*q1p+L12_0sym*q2p+L23_0sym*q3p);

%% W
P0e = simplify(M03sym*P3e);
P0ep = W03_0sym * P0e;

xp_RHS = P0ep(1);  xp_MLF = matlabFunction(xp_RHS,'Vars',[q1 q2 q3 q1p q2p q3p]);  xp_EQ = xp == xp_RHS;
yp_RHS = P0ep(2);  yp_MLF = matlabFunction(yp_RHS,'Vars',[q1 q2 q3 q1p q2p q3p]);  yp_EQ = yp == yp_RHS;
zp_RHS = P0ep(3);  zp_MLF = matlabFunction(zp_RHS,'Vars',[q1 q2 q3 q1p q2p q3p]);  zp_EQ = zp == zp_RHS;

Sp = solve([xp_EQ yp_EQ zp_EQ], [q1p q2p q3p]);

q1p_Sp=simplify(Sp.q1p);
q2p_Sp=simplify(Sp.q2p);
q3p_Sp=simplify(Sp.q3p);

q1p_MLF = matlabFunction(Sp.q1p,'Vars',[q1 q2 q3 xp yp zp]);
q2p_MLF = matlabFunction(Sp.q2p,'Vars',[q1 q2 q3 xp yp zp]);
q3p_MLF = matlabFunction(Sp.q3p,'Vars',[q1 q2 q3 xp yp zp]);
Qpinv = matlabFunction([Sp.q1p;Sp.q2p;Sp.q3p],'Vars',[q1 q2 q3 xp yp zp]);

%% H
W01_0sym = simplify(L01_0sym*q1p);
W12_0sym = simplify(L12_0sym*q2p);
W23_0sym = simplify(L23_0sym*q3p);
W02_0sym = W01_0sym + W12_0sym;

H01_0sym = simplify(L01_0sym*q1pp+L01_0sym^2*q1p^2);
H12_0sym = simplify(L12_0sym*q2pp);
H23_0sym = simplify(L23_0sym*q3pp+L23_0sym^2*q3p^2);

H02_0sym = simplify(H01_0sym + 2*W01_0sym*W12_0sym + H12_0sym);
H03_0sym = simplify(H02_0sym + 2*W02_0sym*W23_0sym + H23_0sym);

P0epp = H03_0sym * P0e;

xpp_RHS = P0epp(1);  xpp_MLF = matlabFunction(xpp_RHS,'Vars',[q1 q2 q3 q1p q2p q3p q1pp q2pp q3pp]);  xpp_EQ = xpp == xpp_RHS;
ypp_RHS = P0epp(2);  ypp_MLF = matlabFunction(ypp_RHS,'Vars',[q1 q2 q3 q1p q2p q3p q1pp q2pp q3pp]);  ypp_EQ = ypp == ypp_RHS;
zpp_RHS = P0epp(3);  zpp_MLF = matlabFunction(zpp_RHS,'Vars',[q1 q2 q3 q1p q2p q3p q1pp q2pp q3pp]);  zpp_EQ = zpp == zpp_RHS;

Spp = solve([xpp_EQ ypp_EQ zpp_EQ], [q1pp q2pp q3pp]);

q1pp_Spp=simplify(Spp.q1pp);
q2pp_Spp=simplify(Spp.q2pp);
q3pp_Spp=simplify(Spp.q3pp);

q1pp_MLF = matlabFunction(Spp.q1pp,'Vars',[q1 q2 q3 q1p q2p q3p xp yp zp xpp ypp zpp]);
q2pp_MLF = matlabFunction(Spp.q2pp,'Vars',[q1 q2 q3 q1p q2p q3p xp yp zp xpp ypp zpp]);
q3pp_MLF = matlabFunction(Spp.q3pp,'Vars',[q1 q2 q3 q1p q2p q3p xp yp zp xpp ypp zpp]);
Qppinv = matlabFunction([Spp.q1pp;Spp.q2pp;Spp.q3pp],'Vars',[q1 q2 q3 q1p q2p q3p xp yp zp xpp ypp zpp]);