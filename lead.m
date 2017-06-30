s= tf('s');
T = 0.01; % 1/10th of fastest time constant
Gp = zpk([],[0,-1,-2],2);

Gz = c2d(Gp,T);
Pm = 55;


%% D(w)
ww1l = 1.2;
[magGww1l, phaseGww1l] = bode(Gz,ww1l);

a0l = 1; % DC gain = 1 for the controller
w0l = 0.1 * ww1l;
wpl = w0l/(a0l*magGww1l);
Dz_lagfl = zpk(-w0l,-wpl,a0l);
Dz_lagl = c2d(Dz_lagfl,T);
Dz_lagGzl = Dz_lagl * Gz;
Cz_lagl = feedback(Dz_lagGzl,1);

step(Cz_lagl);
