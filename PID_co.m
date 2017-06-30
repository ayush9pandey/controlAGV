s= tf('s');
T = 0.01; % 1/10th of fastest time constant
Gp = zpk([],[0,-1,-2],2);

Gz = c2d(Gp,T);
Pm = 55;
ww1l = 1.2;
[magGww1l, phaseGww1l] = bode(Gz,ww1l);
thetad = 180 + Pm - phaseGww1l;
thetar = (thetad * pi)/180;

%% D(w)
Ki = 0.01;
Kp_a = cos(thetar)/magGww1l;
Kd_a = ( (Ki/ww1l) + sin(thetar)/magGww1l ) / ww1l;

Dz_PIDf = Kp_a + ( (Ki*T)/(2*s) ) + ( (Kd_a*s) /( 1 + (T/2)*s )) ;
Dz_PID = c2d(Dz_PIDf,T);
Dz_PIDGz = Dz_PID * Gz;
Cz_PID = feedback(Dz_PIDGz,1);
step(Cz_PID);
