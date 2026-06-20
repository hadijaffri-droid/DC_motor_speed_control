%% Model Parameters
RA = 1.57;        % Armature resistance (Ohm)
LA = 7.8e-3;      % Armature inductance (H)
J = 7.2e-3;       % Moment of inertia (kg m^2)
kG = 0.126;       % Motor/Generator constant (Vs/rad)
alpha_F = 0.0002844;  % Viscous friction (Nms/rad)
tau_F0 = 0.04631;     % Dry friction torque (Nm)


%% Sampling time, filters etc.
Ts = 0.001;           % Sample time (s)
Tfn = 0.02;           % Speed filter time constant (s)
Tfi = 0.002;          % Current filter time constant (s)
s = tf('s');          % Complex variable s
Gfn = 1/(1+Tfn*s);   % Speed filter
Gfi = 1/(1+Tfi*s);   % Current filter
G_RL = 1/(LA*s + RA);  % Model of the RL circuit

%% Current Controller (Modulus Optimum)
T1 = LA/RA;                % Larger time constant
Tsigma = Tfi + 0.5*Ts;    % Equivalent small time constant
KS = 1/RA;                 % Plant gain

Tii = T1;                  % Controller integral time
Kri = LA/(2*Tfi + Ts);     % Controller gain

Gri = Kri*(1 + 1/(Tii*s));   % Current controller transfer function

%% Speed Controller (Symmetrical Optimum)
Gri_tf = Kri*(1+1/(Tii*s));          % Current controller
Gi = minreal(Gri_tf*G_RL/(1+Gri_tf*G_RL*Gfi), 1e-4);  % Inner loop
Gm = minreal(1/J/s/(1+ alpha_F/J/s));                   % Mechanics
Gn = minreal(Gi*kG*Gm/(1+Gi*kG*Gm*kG/Gri_tf)*30/pi, 1e-2); % Speed
Gsn = Gn*Gfn;                         % Speed + filter

% Start sisotool to find Krn and Tin
sisotool(Gsn, (1+1/s));

%% Speed Controller Parameters
Krn = 0.1275;
Tin = 1/11.03;

%% Anti-windup gains
Kawi = Kri;
Kawn = 50*Krn;


La=LA;
Ra=RA;
