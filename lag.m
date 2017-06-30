s= tf('s');
T = 0.01; % 1/10th of fastest time constant
Gp = zpk([],[0,-1,-2],2);

Gz = c2d(Gp,T);
Pm = 55;
ww1 = 0.36;

[magGww1, phaseGww1] = bode(Gz,ww1);


%% D(w)
a0 = 1; % DC gain = 1 for the controller
w0 = 0.1 * ww1;
wp = w0/(a0*magGww1);
Dz_lagf = zpk(-w0,-wp,a0);
Dz_lag = c2d(Dz_lagf,T);
Dz_lagGz = Dz_lag * Gz;
Cz_lag = feedback(Dz_lagGz,1);
step(Cz_lag);
