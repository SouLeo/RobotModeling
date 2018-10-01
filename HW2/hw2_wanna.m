%%% Selma Wanna
%%% Homework #2
%%% Dr. James Sulzer

% %% initialize the DH Table for RRP Mechanism
% num_of_links = 3;
% is_revolute_joints = [1 1 0];
% a_vector = [0 0 0];
% alpha_vector = [pi/2 pi/2 0];
% %%
% 
% RRP = ForwardKinematics(num_of_links, is_revolute_joints, a_vector, alpha_vector); % RRP robot mechanism
% RRP = RRP.add_offset_to_joint_variable(2, pi/2);
% RRP = RRP.set_joint_param(1, 2, 2); % on the first link, add a constant d offset of 2
% RRP = RRP.set_joint_param(3, 2, 10); % on the third link, add a constant d offset of 10
% 
% %% Robot will update from time t_start to t_stop
% 
% t_stop = 200;
% 
% for t = 1:t_stop
%     RRP = RRP.create_A_matrices;
%     RRP = RRP.base_to_joints_transformations();
%     RRP = RRP.draw_lines();
%     RRP = RRP.update_joint_variables();
%     pause(0.2)
% end
% 
% disp('hw 2, rrp mechanism, job done')
% 
% clear
% clc
% close all

%% initialize the DH Table for 7DOF Mechanism
num_of_links = 7;
is_revolute_joints = [1 1 1 1 1 1 1];
a_vector = [0 0 0 0 0 0 0];
alpha_vector = [-pi/2 -pi/2 pi/2 pi/2 -pi/2 -pi/2 0];
%%

Seven_DOF = ForwardKinematics(num_of_links, is_revolute_joints, a_vector, alpha_vector); % 7DOF robot mechanism
Seven_DOF = Seven_DOF.add_offset_to_joint_variable(2, -pi/2);
Seven_DOF = Seven_DOF.add_offset_to_joint_variable(3, -pi/2);
Seven_DOF = Seven_DOF.add_offset_to_joint_variable(6, -pi/2);
Seven_DOF = Seven_DOF.set_joint_param(4, 2, 5); % on the first link, add a constant d offset of 2
Seven_DOF = Seven_DOF.set_joint_param(5, 2, 5); % on the third link, add a constant d offset of 10

%% Robot will update from time t_start to t_stop

t_stop = 100;

for t = 1:t_stop
    Seven_DOF = Seven_DOF.create_A_matrices;
    Seven_DOF = Seven_DOF.base_to_joints_transformations();
    Seven_DOF = Seven_DOF.draw_lines();
    Seven_DOF = Seven_DOF.update_joint_variables();
    pause(0.2)
end

disp('hw 2, 7dof mechanism, job done')


