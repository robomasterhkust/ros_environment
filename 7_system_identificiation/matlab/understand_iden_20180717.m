% system
clc; clear,close all; 
s = tf('s');
z = tf('z', 0.001);

%% 20180717, identify system in s domain
% close_loop_pitch_vel_zpk = ...
%     zpk([-30.294 + 29.693i; -30.294 - 29.693i; -0.03264],...
%         [-14.348 + 23.190i; -14.348 - 23.190i; -2.2055], ...
%            0.3484);
close_loop_pitch_vel = tf([27.69, 1.101],...
                        [1,28.46, 143.2]);

close_loop_pitch_pos = tf([190.47,4836.3], ...
                            [1,133.58,4920.2]);
  
close_loop_pitch_vel_z = c2d(close_loop_pitch_vel, 0.001);
close_loop_pitch_pos_z = c2d(close_loop_pitch_pos, 0.001);


% backward difference discretization
z_pitch_vel  = -1200 - 16 * z / (z - 1);
z_pitch_atti = 60 + 1.5 * z / (z - 1) - 0.5 * (z-2+1/z) / (z - 1);
  
% figure;
% bode(d_pitch_vel_zpk, {0.1, 1000});
% title("pitch vel plant Bode");

% figure;
% bode(open_loop_pitch_vel, {0.1, 1000});
% title("pitch vel open loop Bode");
% [Gm_p_vel,Pm_p_vel,Wgm_p_vel,Wpm_p_vel] = margin(open_loop_pitch_vel);


%% check with delayed system
delay_close_loop_pitch_vel = close_loop_pitch_vel * exp(-0.004 * s);
delay_close_loop_pitch_pos = close_loop_pitch_pos * exp(-0.022 * s);
delay_close_loop_pitch_vel_z = c2d(delay_close_loop_pitch_vel, 0.001);
        % delay_close_loop_pitch_pos_z = c2d(delay_close_loop_pitch_pos, 0.001);

op_pitch_pos = z_pitch_atti * delay_close_loop_pitch_vel_z;
cl_pitch_pos = op_pitch_pos / (1 + op_pitch_pos);
cl_pitch_pos_reduced = minreal(cl_pitch_pos, 1e-1);


%% 20180718,  pitch velocity and position open loop after removing the mean
open_loop_vel_z = tf([0,0.02472,-0.02471],   [1,-1.998,0.9984], 0.001);
open_loop_pos_z = tf([0,0.001708,-0.001708], [1,-2.976,2.953,-0.9764], 0.001, 'IODelay', 9);
close_loop_vel_z= tf([0,0.02869,-0.02869],   [1,-1.9695,0.9696], 0.001, 'IODelay', 6);
tf_close_loop_pos_z= tf([0,0.0027956],       [1,-1.9714,0.97427], 0.001);
delay_close_loop_pos_z = tf([1],[1],0.001,'IODelay', 8);
close_loop_pos_z= tf_close_loop_pos_z * delay_close_loop_pos_z;

close_loop_vel_z_derived = open_loop_vel_z / (1+open_loop_vel_z);
close_loop_pos_z_derived = open_loop_pos_z / (1+open_loop_pos_z);

H_ideal_observer  = tf([1],[1],0.001,'IODelay', 0);
H_visual_observer = tf([1],[1],0.001,'IODelay', 45);
H_pade = pade(H_visual_observer, 2);
G_pade = pade(close_loop_vel_z, 3);
C_visual_pitch = pidstd(3);
C_visual_yaw   = pidstd(3);
% sisotool(G_pade * C_visual_pitch, 1, H_pade, 1);

%         figure;
%         bode(open_loop_pos_z, close_loop_vel_z * z_pitch_atti, {0.1, 1571}, 'r--');
%         title("verify the similarity between open and close loop, velocity");
% 
%         figure;
%         bode(close_loop_pos_z, close_loop_pos_z_derived, {0.1, 1571}, 'r--');
%         title("verify the similarity between open and close loop, position");
%         % result: close loop identification for position not trustworth

%% Start the Smith Predictor 20180720
% Filter, capture low-frequency disturbances
%    F = 1/20 * s + 1;
F = 5e-05 / (z - 1);
% F = tf([1],[1],0.001);
F.InputName = 'dy';
F.OutputName= 'dp';

% Process
P = close_loop_vel_z * H_visual_observer;
P.InputName  = 'u';
P.OutputName = 'y0';

% Prediction model
Gp = close_loop_vel_z;
Gp.InputName = 'u';
Gp.OutputName= 'yp';

Dp = H_visual_observer;
Dp.InputName = 'yp';
Dp.OutputName= 'y1';

% Overall plant
% S1 = sumblk('ym = yp + dp');
% S2 = sumblk('dy = y0 - y1');
% Plant = connect(P, Gp, Dp, F, S1, S2, 'u', 'ym');

C = pidstd(1);
C.InputName = 'e';
C.OutputName = 'u';

% compare the system with the non-compensated
Sum1 = sumblk('e = ysp - yp - dp');
Sum2 = sumblk('y = y0 + d');
Sum3 = sumblk('dy = y - y1');
T = connect(P, Gp, Dp, C, F, Sum1, Sum2, Sum3, {'ysp', 'd'}, 'y');

T_origin = feedback([close_loop_vel_z * H_visual_observer, 1], C, 1, 1);
T_origin.InputName = {'ysp' 'd'};

figure;
step(T, 'b', T_origin, 'r--'), grid on   
legend('Smith Predictor', 'PI Controller');  

% figure;
% bode(T(1, 1), 'b', T_origin(1, 1), 'r--', {0.1, 628})
% grid on
% legend('Smith Predictor', 'PI Controller');

%%         %% New construction of the Smith predictor, 20180721, using accumulation
%         F = 5e-05 / (z - 1);
%         F.InputName = 'dy';
%         F.OutputName= 'dp';
% 
%         % Process, change to position, and take an derivative on the top
%         P = close_loop_pos_z * H_visual_observer;
%         P.InputName  = 'u';
%         P.OutputName = 'y0';
% 
%         % Prediction model
%         Gp = tf_close_loop_pos_z;
%         Gp.InputName = 'u';
%         Gp.OutputName= 'yp';
% 
%         Dp = H_visual_observer * delay_close_loop_pos_z;
%         Dp.InputName = 'yp';
%         Dp.OutputName= 'y1';
% 
%         C = pidstd(5);
%         C.InputName = 'e';
%         C.OutputName = 'u';
% 
%         % compare the system with the non-compensated
%         Sum1 = sumblk('e = ysp - yp - dp');
%         Sum2 = sumblk('y = y0 + d');
%         Sum3 = sumblk('dy = y - y1');
%         T = connect(P, Gp, Dp, C, F, Sum1, Sum2, Sum3, {'ysp', 'd'}, 'y');
% 
%         T_origin = feedback([close_loop_pos_z * H_visual_observer, 1], C, 1, 1);
%         T_origin.InputName = {'ysp' 'd'};
% 
% 
%         figure;
%         step(T, 'b'), grid on       % , T_origin, 'r--'
%         legend('Smith Predictor');  % , 'PI Controller'
% 
%         figure;
%         bode(T(1, 1), 'b', T_origin(1, 1), 'r--', {0.1, 628});
%         grid on
%         legend('Smith Predictor', 'PI Controller');

%% Yaw axis, 20180719
open_loop_yaw_vel_z = tf([0,0.0477816911823554,-0.0476449885958894],   [1,-1.99663948611521,0.996672525083871], 0.001, 'IODelay', 7);
open_loop_yaw_pos_z = tf([0,0.00233761189719394,-0.00228286004870625], [1,-1.26168781182833,-0.354593650550658,0.622153683100762], 0.001, 'IODelay', 76);
close_loop_yaw_vel_z= tf([0,0.0556869716532869,-0.0557230395559041],   [1,-1.94124749980598,0.941312329421924], 0.001, 'IODelay', 8);
close_loop_Yaw_pos_z= tf([0,0.00222886852552263,-0.00217513201453841], [1,-1.26097729116364,-0.365814123972705,0.632560029662831], 0.001, 'IODelay', 76);

close_loop_yaw_vel_z_derived = open_loop_yaw_vel_z / (1+open_loop_yaw_vel_z);
close_loop_yaw_pos_z_derived = open_loop_yaw_pos_z / (1+open_loop_yaw_pos_z);
%     figure;
%     bode(close_loop_yaw_vel_z, close_loop_yaw_vel_z_derived, {0.1, 1571}, 'r--');
%     title("verify the similarity between open and close loop, velocity");
% 
%     figure;
%     bode(close_loop_Yaw_pos_z, close_loop_yaw_pos_z_derived, {0.1, 1571}, 'r--');
%     title("verify the similarity between open and close loop, position");

%% history
%         pzmap(cl_pitch_pos, cl_pitch_pos_reduced)
%         grid on
%         figure;
%         bode(delay_close_loop_pitch_vel_z, close_loop_pitch_vel_z, {0.1, 1000});
%         title("delayed pitch close loop for velocity");
% 
%         figure;
%         bode(delay_close_loop_pitch_pos, close_loop_pitch_pos, {0.1, 1000});
%         title("delayed pitch close loop for position");
% 
%         figure;
%         bode(delay_close_loop_pitch_pos_z, cl_pitch_pos_reduced, {0.1, 1000}, 'r--');
%         title("comparasion between expriment and theory");
% 
%         % reverse to find the plant
%         open_loop_pitch_pos = delay_close_loop_pitch_pos_z / (1 - delay_close_loop_pitch_pos_z);
%         pitch_pos_plant = open_loop_pitch_pos / z_pitch_atti;
%         pitch_pos_plant_reduced = minreal(pitch_pos_plant, 1e-2);
%         figure;
%         % pzmap(pitch_pos_plant, pitch_pos_plant_reduced)
%         % grid on
%                 % result the close loop identification generate system
%                 % too complex to solve
% 
%         bode(delay_close_loop_pitch_vel_z, pitch_pos_plant_reduced, {0.1, 1000}, 'r--');
%         title("inverse comparasion between expriment and theory");
%         open_loop_vel_z = tf([0,0.02560239,-0.02559506],[1,-1.998026,0.998094],0.001,...
%                             'IODelay', 6);
%         open_loop_vel = tf([25.54468,7.155320], [1,1.69757,71.42746], ...
%                             'IODelay', 0.006);
%         figure;
%         bode(open_loop_vel, open_loop_vel_z, {0.1, 157.1}, 'r--');
%         title("system iden in s and z transform");
%                 % result: identify discrete system directly
