%%
% clc
% clear
% s=tf("s");
% 
% j= 0.01;
% tf_cubesat= tf(1 , [j 0 0]);
% Kp = 0.0035705;
% Ki = 7.8322e-05;
% Kd = 0.0136;
% 
% % Pid_Tf= Kp + Ki/s + Kd*s
% Pid_Tf = pid(Kp,Ki, Kd);
% Tf_OL = tf_cubesat*Pid_Tf;
% Tf_CL = feedback(Tf_OL, 1);
% 
% step(Tf_CL)

