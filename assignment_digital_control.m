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

%Find D(w)

%% Lag compensator, ww1 = 0.36 
a0 = 1; % DC gain = 1 for the controller
w0 = 0.1 * ww1;
wp = w0/(a0*magGww1);
Dz_lagf = zpk(-w0,-wp,a0);
Dz_lag = c2d(Dz_lagf,T);
Dz_lagGz = Dz_lag * Gz;
Cz_lag = feedback(Dz_lagGz,1);
% 
%% Lead compensator, ww1l = 1.2
ww1l = 1.2;
[magGww1l, phaseGww1l] = bode(Gz,ww1l);
thetadl = 180 + Pm - phaseGww1l;
thetarl = (thetadl * pi)/180;
a0l = 1; % DC gain = 1 for the controller
w0l = 0.1 * ww1l;
wpl = w0l/(a0l*magGww1l);
Dz_lagfl = zpk(-w0l,-wpl,a0l);
Dz_lagl = c2d(Dz_lagfl,T);
Dz_lagGzl = Dz_lagl * Gz;
Cz_lagl = feedback(Dz_lagGzl,1);

%% PI Controller, ww1 = 0.36

Kp1 = cos(thetar)/magGww1;
Ki1 = (-ww1 * sin(thetar) )/ magGww1;
Dz1f = Kp1 + ((Ki1 * T)/(2*s));
Dz1 = c2d(Dz1f,T);
Dz1Gz = Dz1 * Gz;
Cz1 = feedback(Dz1Gz,1);

%% PID Controller, ww1l = 1.2
Ki = 0.01;
Kp_a = cos(thetarl)/magGww1l;
Kd_a = ( (Ki/ww1l) + sin(thetarl)/magGww1l ) / ww1l;

Dz_PIDf = Kp_a + ( (Ki*T)/(2*s) ) + ( (Kd_a*s) /( 1 + (T/2)*s )) ;
Dz_PID = c2d(Dz_PIDf,T);
Dz_PIDGz = Dz_PID * Gz;
Cz_PID = feedback(Dz_PIDGz,1);

 
% %% Bandlimited PID
% disp('Bandlimited PID');
% Kd = ( ( (2/T)^2 ) + ww1^2 ) * ( (T/2)^2 ) * (1/ww1) * ( (Ki/ww1) + (sin(thetar)/magGww1) );
% Kp = ( cos(thetar)/magGww1 ) - ( (Kd*(ww1^2) * ((2/T))) / (( (2/T)^2 ) + ww1^2));
% Dz_PID = Kp + (( (Ki*T)/2 ) * tf([1 1],[1 -1],T) ) + ( (Kd/T)*tf([1 -1],[1 0],T) );
% Dz_PIDGz = Dz_PID * Gz;
% Cz = feedback(Dz_PIDGz,1);
% disp('Gain and Phase');
% [Gm,Pm] = margin(Dz_PIDGz);
% GmdB = 20*log10(Gm);
% 
% 
% 
%% Deadbeat Controller
% % Desired closed loop TF
% Gcl = tf(1.369,[1 2.027 1.369]);
% Fz1 = c2d(Gcl,T);
% % zpk(Gdz);
% 
% %Obtain Dz - controller TF, analytically
% Dz1 = Fz1/(Gz*(1-Fz1));
% zpk(Dz1);
% minreal(zpk(Dz1))


% Finite settling time controller
% Fz = a0 + a1 z^-1 + a2 z^-2 + ... a0 = 0 (from Gz expression, the first term
% is z^-1). N = 2

Fz2 = tf(1,[1 0 0 0],T);
Dz2 = Fz2/(Gz*(1-Fz2));
Dz2Gz = Dz2 * Gz;
Cz2 = feedback(Dz2Gz,1);
 
%% Plot transient step response

hold on
step(Cz_lag,'r');
step(Cz_lagl,'b');
step(Cz1,'m');
step(Cz_PID,'y');
%step(Cz2,'k');
legend('Lag Compensator','Lead Compensator','PI Controller','PID Controller');
