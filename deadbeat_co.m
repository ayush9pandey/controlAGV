s= tf('s');
T = 0.01; % 1/10th of fastest time constant
Gp = zpk([],[0,-1,-2],2);

Gz = c2d(Gp,T);
%% D(w)
Fz2 = tf(1,[1 0.0088 3.9478e-5],T); % N = 3;
Dz2 = Fz2/(Gz*(1-Fz2));
Dz2Gz = Dz2 * Gz;
Cz2 = feedback(Dz2Gz,1);
step(Cz2);
 
