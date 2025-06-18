%% A
% clc
% clear


%% Plant Parameters
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

% State Space Model


% Step
step(BLDCM_tf)
stepinfo(BLDCM_tf, 'SettlingTimeThreshold', 0.05)
grid on

%% Control







