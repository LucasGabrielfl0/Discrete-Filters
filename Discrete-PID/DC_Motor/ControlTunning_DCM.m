%% A
% clc
% clear
s= tf('s');

%% Parameters
% Measured Parameters
R  = 1.2/2;         % ohm
L  = 10/2;           % H

% Constants
Ke = 0.296;         % V/Rad/s
Kt = 0.3611;        % Nm/A

% Guessed / Given with doubt
J  = 7.78*10^-7;    % Kg-m^2
D  = 0.003;         % Kg-m-s / rad  
Vin = 100;          % V


BLDCM_tf= Kt*Vin/( (L*s+R)*(J*s+D) + (Kt*Ke));

%% Step
step(BLDCM_tf)
stepinfo(BLDCM_tf, 'SettlingTimeThreshold', 0.05)
grid on

%% Control
% Kp = 0.0074385;
% Ki = 0.053277;
Kp = 0.0010278;
Ki = 0.0078311;
Kd = 0;
Ts = 1e-6;
PI_tf = pid(Kp, Ki);

CLTF = feedback(BLDCM_tf*PI_tf, 1);
step(CLTF)
stepinfo(CLTF, 'SettlingTimeThreshold', 0.05)


%% Discrete Control
a0= ( Kp + (Ts*Ki/2) );
a1= ( (-Kp) + (Ts*Ki/2) );

Pidz2 =tf([a0 a1], [1 -1], Ts);
Pidz = c2d(PI_tf,Ts,'tustin');
BLDCM_tfz = c2d(BLDCM_tf, Ts, 'tustin');
CLTF_Z = feedback (BLDCM_tfz*Pidz2, 1);
step(100*CLTF_Z, 3);
grid on



BLDC_Num= cell2mat(BLDCM_tfz.Numerator);
BLDC_Den= cell2mat(BLDCM_tfz.Denominator);



