s= tf('s');
T = 0.01; % 1/10th of fastest time constant
Gp = zpk([],[0,-1,-2],2);

Gz = c2d(Gp,T);
Pm = 55;
ww1 = 0.36;
%Ki = 0.01;
[magGww1, phaseGww1] = bode(Gz,ww1);
thetad = 180 + Pm - phaseGww1;
thetar = (thetad * pi)/180;

%% D(w)
Kp1 = cos(thetar)/magGww1;
Ki1 = (-ww1 * sin(thetar) )/ magGww1;
Dz1f = Kp1 + ((Ki1 * T)/(2*s));
Dz1 = c2d(Dz1f,T);
Dz1Gz = Dz1 * Gz;
Cz1 = feedback(Dz1Gz,1);
step(Cz1)
