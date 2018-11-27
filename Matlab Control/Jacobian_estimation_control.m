% The initial code for a Jacobian-based approach to controlling a
% soft-robot in ROS. This attempt was successful, but far to slow to record
% any useful data.
% Final version completed in Octobebr 2017
% Written by Avinash  Soor
% Git: Avinasho
% Written for the MEng Individual Project


clear; rosshutdown; clc; close all;
rosinit;
global pos_curr t_curr quat_curr q_curr_pos
aurSub = rossubscriber('/aurora',@auroraCallback);
curr_pos_Sub = rossubscriber('/curr_pos', @curr_posCallback);
pub_pos = rospublisher( 'cmd_vel', 'geometry_msgs/Point32' );
pos_mes = rosmessage( pub_pos );
error_pub = rospublisher('/Error',  'geometry_msgs/Point32' );
err_mes = rosmessage( error_pub);

load('kinematics.mat');
K = K_global(3:5, 3:5); % Orientation
H = H(3:5, 1:3); % orientation

i = 1;
Kp = 0.001;
stopNow = false;
Pg = [0, 0, -5]; % x, y, z for position % z, x, y for orientation

eul = zeros(3,size(4,2));
R_tip_tracker = rotz(180)*roty(-90);
d_tip_tracker = [-35; -7; -47];
T_tip_tracker = [R_tip_tracker, d_tip_tracker; zeros(1,3),1];
% roslaunch kinematics pfMatlab.launch 

pos_mes.X = 1;
pos_mes.Y = 1;
pos_mes.Z = 1;
send(pub_pos, pos_mes)

while stopNow == false % While all the errors are above the tolerance
    
    aurPos = [pos_curr(1), pos_curr(2), pos_curr(3)]; % current aurora pos
    q_Pos = [q_curr_pos(1), q_curr_pos(2), q_curr_pos(3)];
    quatPos = [quat_curr(1), quat_curr(2), quat_curr(3), quat_curr(4)]; % x, y, z, w
    
    ALL_aurPos(1:3, i) = aurPos;
    ALL_quatPos(1:4, i) = quatPos; 
    
    if i == 1
        Q0 = quatPos; % Initial quaternian values
        T0 = t_curr; % Saves initial time to correct for offset
        R0 = quat2rotm(Q0); % Changes quat to rotation
        d0 = aurPos; % The current position 
        Tr0 = zeros(4,4);
        Tr0(1:3,1:3) = R0;
        Tr0(1:3,4) = d0;
        Tr0_offset = invT(Tr0 * T_tip_tracker);
    end
   
    R1 = quat2rotm( quatPos );
    d1 = aurPos'; % aurora position
    T1 = [R1, d1; zeros(1, 3), 1];
    T_total = Tr0_offset * T1 * T_tip_tracker;
    
    eul = rotm2eul( T_total(1:3, 1:3) )' * 180/pi;
    eulx = eul(3, :);
    eulz = eul(1, :);
    eul(3, :) = eulz;
    eul(1, :) = eulx;
    aurPos = T_total(1:3, 4)'; % tip position
    
    ALL_t_aurPos(1:3, i) = aurPos;
    ALL_eul(1:3, i) = eul;
    Pg_all(1:3, i) = Pg;
    ALL_q(1:3, i) = q_Pos;
    
    time(i) = t_curr - T0;
    ERR(1:3, i) = Pg - aurPos; % Work out the error and saves to an array
    
    if i < 20
        dq = [1, 1, 1]';
        dx = [1, 1, 1]';
        J_prev = eye(3);
        J_est = eye(3);
    else
        dq = ALL_q(1:3, i-1) - q_Pos';
        dx = ALL_t_aurPos(1:3, i-1) - aurPos';
        J_prev = J_est;
        J_est = J_estimation(J_prev, dq, dx);
    end
    
    err_mes.X = ERR(1, i);
    err_mes.Y = ERR(2, i);
    err_mes.Z = ERR(3, i);
	send(error_pub, err_mes)
 
    dq_est = inv(J_est) * ERR(1:3, i);
    
    ALL_dq_est(1:3, i) = dq_est;
    
    if i < 20
        pos_mes.X = 0.1;
        pos_mes.Y = 0.1;
        pos_mes.Z = 0.1;
    else
        pos_mes.X = dq_est(1);
        pos_mes.Y = dq_est(2);
        pos_mes.Z = dq_est(3);
    end
    send(pub_pos, pos_mes)
 
    pause(0.001)
    i = i+1;
end

function T_inv = invT(T)
    R = T(1:3,1:3);
    d = T(1:3,4);
    T_inv = zeros(4,4);
    
    T_inv(1:3,1:3) = R';
    T_inv(1:3,4) = -R'*d;
end

function J_est = J_estimation(J_prev, dq, dx)

    kappa = 1;

    J_est = (kappa*(((dx-(J_prev*dq))*dq')/(dq'*dq)))+J_prev;
    
end
 
function curr_posCallback(~, msg)
global q_curr_pos
q_curr_pos = [msg.X, msg.Y, msg.Z];
end

function auroraCallback(~, msg)
global pos_curr q_curr t_curr quat_curr
quat_curr = [msg.Poses.Orientation.X; msg.Poses.Orientation.Y; msg.Poses.Orientation.Z; msg.Poses.Orientation.W];
pos_curr = [msg.Poses.Position.X;msg.Poses.Position.Y;msg.Poses.Position.Z];
q_curr = [msg.Poses.Orientation.W,msg.Poses.Orientation.X,msg.Poses.Orientation.Y,msg.Poses.Orientation.Z]';
t_curr = msg.Header.Stamp.Sec+msg.Header.Stamp.Nsec*10^(-9);
end