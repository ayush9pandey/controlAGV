%Open loop TF
Gp = tf(10,[1 11 10]);
T = 0.01; % 1/10th of fastest time constant
Gp2 = tf([1],[1 1]);
% hold on
% step(Gp,'r');
% step(Gp2,'b');

Gz = c2d(Gp,T);
zpk(Gz);

% % Desired closed loop TF
Gcl = tf(1.369,[1 2.027 1.369]);
Fz1 = c2d(Gcl,T);
% zpk(Gdz);

%Obtain Dz - controller TF, analytically
Dz1 = Fz1/(Gz*(1-Fz1));
zpk(Dz1);
minreal(zpk(Dz1))


% Finite settling time controller
% Fz = a0 + a1 z^-1 + a2 z^-2 + ... a0 = 0 (from Gz expression, the first term
% is z^-1). N = 2

Fz2 = tf(1,[1 0],T);
Dz2 = Fz2/(Gz*(1-Fz2));
zpk(Dz2);
minreal(zpk(Dz2))

% A = [a11 a12;a21 a22];
% B = [b1;b2];
% phi = eye(2) + A*T + ((A^2)*(T^2))/2;
% si = 

