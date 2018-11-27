% Resets the initial position of a soft-robot using ROS
% Final version completed in Octobebr 2017
% Written by Avinash  Soor
% Git: Avinasho
% Written for the MEng Individual Project

clear; rosshutdown; clc;
rosinit

pub_pos = rospublisher( 'cmd_vel', 'geometry_msgs/Point32' );
pos_mes = rosmessage( pub_pos );

pos_mes.X = 0;
pos_mes.Y = 0;
pos_mes.Z = 0;
send(pub_pos, pos_mes)
pause(0.005)
pos_mes.X = -100;
pos_mes.Y = -100;
pos_mes.Z = -100;
send(pub_pos, pos_mes)
pause(5)
pos_mes.X = 0;
pos_mes.Y = 0;
pos_mes.Z = 0;
send(pub_pos, pos_mes)
pause(0.005)
pos_mes.X = 1;
pos_mes.Y = 1;
pos_mes.Z = 1;
send(pub_pos, pos_mes)
pause(0.7)    
pos_mes.X = 0;
pos_mes.Y = 0;
pos_mes.Z = 0;
send(pub_pos,pos_mes)