% read pitch data
clc; clear;close all;
fileID = fopen(fullfile('/Users/beck/Google Drive/MATLAB', 'Ozone_DataGraph_180715_pitch1.csv'));
C_pitch = textscan(fileID, '%s %s %s %s %s', 1);
raw_data_pitch = textscan(fileID, '%d %f %f %f %f', 'Delimiter', ',');
fclose(fileID);

microsec          = raw_data_pitch{1};
pitch_speed       = raw_data_pitch{3};
pitch_speed_cmd   = raw_data_pitch{4};
pitch_atti_cmd    = raw_data_pitch{5};
pitch_euler_angle = raw_data_pitch{2};

%%% preprocess data
nonzero_index = 25001;
nonzero_index_end = 84000;

microsec = microsec(nonzero_index : nonzero_index_end) - microsec(nonzero_index);
pitch_speed       = pitch_speed(nonzero_index : nonzero_index_end);
pitch_speed_cmd   = pitch_speed_cmd(nonzero_index : nonzero_index_end);
pitch_atti_cmd    = pitch_atti_cmd(nonzero_index : nonzero_index_end);
pitch_euler_angle = pitch_euler_angle(nonzero_index : nonzero_index_end);


%% remove drift
peaks_1hz = 10;
peaks_normal = 40;

% [pitch_atti_normal, pitch_atti_cmd_normal]= ...
%     system_iden_remove_mean(microsec, pitch_speed, pitch_speed_cmd, pitch_euler_angle, pitch_atti_cmd, peaks_1hz, peaks_normal);
% % TODO: align the two array size
% pitch_atti_err  = pitch_atti_err(1: 55000);
% pitch_atti_normal  = pitch_atti_normal(1: 55000);

tt   = double([ones(length(microsec), 1) microsec]);
b1   = tt \ pitch_euler_angle;
b2   = tt \ pitch_atti_cmd;
b3   = tt \ pitch_speed;
b4   = tt \ pitch_speed_cmd;
pitch_angle_normal = pitch_euler_angle - tt * b1;
pitch_cmd_normal   = pitch_atti_cmd - tt * b2;
pitch_speed_normal = pitch_speed - tt * b3;
pitch_speed_cmd_normal = pitch_speed_cmd - tt * b4;

pitch_speed_err = pitch_speed_cmd_normal - pitch_speed_normal;
pitch_atti_err  = pitch_cmd_normal - pitch_angle_normal;

        % result: pitch_speed effect is little; pitch_att effect is strong

%% plot

figure;
subplot(2, 2, 1);
plot(microsec, pitch_speed);
title('pitch speed');
ylabel('radian/s');

subplot(2, 2, 2);
plot(microsec, pitch_speed_cmd);
title('pitch speed command');
ylabel('radian/s');

subplot(2, 2, 3);
plot(microsec, pitch_cmd_normal);
title('pitch attitude command');
ylabel('radian');

subplot(2, 2, 4);
plot(microsec, pitch_angle_normal);
title('pitch Euler angle');
ylabel('radian');
xlabel('t (microseconds)');

% read yaw data; yaw1, yaw2 has camera on; yaw3, yaw4 might have wire drift
fileID = fopen(fullfile('/Users/beck/Google Drive/MATLAB', 'Ozone_DataGraph_180715_yaw5.csv'));
C_yaw = textscan(fileID, '%s %s %s %s %s', 1);
raw_data_yaw = textscan(fileID, '%d %f %f %f %f', 'Delimiter', ',');
fclose(fileID);

microsec_yaw    = raw_data_yaw{1};
yaw_speed       = raw_data_yaw{2};
yaw_speed_cmd   = raw_data_yaw{3};
yaw_atti_cmd    = raw_data_yaw{4};
yaw_euler_angle = raw_data_yaw{5};

%%% preprocess data
nonzero_index = 10001;
nonzero_index_end = 84000;
microsec_yaw    = microsec_yaw(nonzero_index : nonzero_index_end) - microsec_yaw(nonzero_index);
yaw_speed       = yaw_speed(nonzero_index : nonzero_index_end);
yaw_speed_cmd   = yaw_speed_cmd(nonzero_index : nonzero_index_end);
yaw_atti_cmd    = yaw_atti_cmd(nonzero_index : nonzero_index_end);
yaw_euler_angle = yaw_euler_angle(nonzero_index : nonzero_index_end);

tt   = double([ones(length(microsec_yaw), 1) microsec_yaw]);
b5   = tt \ yaw_euler_angle;
b6   = tt \ yaw_atti_cmd;
b7   = tt \ yaw_speed;
b8   = tt \ yaw_speed_cmd;
yaw_angle_normal = yaw_euler_angle - tt * b5;
yaw_cmd_normal   = yaw_atti_cmd - tt * b6;
yaw_speed_normal = yaw_speed - tt * b7;
yaw_speed_cmd_normal = yaw_speed_cmd - tt * b8;


yaw_speed_err = yaw_speed_cmd_normal - yaw_speed_normal;
yaw_atti_err  = yaw_cmd_normal - yaw_angle_normal;

%%% plot
figure;
subplot(2, 2, 1);
plot(microsec_yaw, yaw_speed);
title('yaw speed');
ylabel('radian/s');

subplot(2, 2, 2);
plot(microsec_yaw, yaw_speed_cmd);
title('yaw speed command');
ylabel('radian/s');

subplot(2, 2, 3);
plot(microsec_yaw, yaw_cmd_normal);
title('yaw attitude command');
ylabel('radian');

subplot(2, 2, 4);
plot(microsec_yaw, yaw_angle_normal);
title('yaw Euler angle');
ylabel('radian');
xlabel('t (microseconds)');