dT_ini = dT;
dT_target = 0.001;
nPoints_target = floor(dT*nPoints/dT_target);

tt_fine = linspace(tt(1),tt(end),nPoints_target);

Q1_interp = interp1(tt,Q(1,:),tt_fine);
Q2_interp = interp1(tt,Q(2,:),tt_fine);
Q3_interp = interp1(tt,Q(3,:),tt_fine);

Q_interp = [Q1_interp; Q2_interp; Q3_interp];

Q1p_interp = interp1(tt,Qp(1,:),tt_fine);
Q2p_interp = interp1(tt,Qp(2,:),tt_fine);
Q3p_interp = interp1(tt,Qp(3,:),tt_fine);

Qp_interp = [Q1p_interp; Q2p_interp; Q3p_interp];

Q1pp_interp = interp1(tt,Qpp(1,:),tt_fine);
Q2pp_interp = interp1(tt,Qpp(2,:),tt_fine);
Q3pp_interp = interp1(tt,Qpp(3,:),tt_fine);

Qpp_interp = [Q1pp_interp; Q2pp_interp; Q3pp_interp];

Fq1_interp = interp1(tt,Fq(1,:),tt_fine);
Fq2_interp = interp1(tt,Fq(2,:),tt_fine);
Fq3_interp = interp1(tt,Fq(3,:),tt_fine);

Fq_interp = [Fq1_interp; Fq2_interp; Fq3_interp];