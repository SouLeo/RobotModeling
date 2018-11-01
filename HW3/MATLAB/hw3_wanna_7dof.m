%%% Selma Wanna
%%% Homework #3
%%% Dr. James Sulzer

%% initialize the DH Table for RRR Mechanism

is_revolute_joints = [1 1 1 1 1 1 1];
num_of_links = length(is_revolute_joints);
a_vector = [0 0 0 0 0 0 0];
alpha_vector = [-pi/2 -pi/2 pi/2 pi/2 -pi/2 -pi/2 0];
%% Forward Kinematics Setup

seven_dof = ForwardKinematics(num_of_links, is_revolute_joints, a_vector, alpha_vector); % 7Dof robot mechanism
seven_dof = seven_dof.add_offset_to_joint_variable(2, 0);
seven_dof = seven_dof.add_offset_to_joint_variable(3, -pi/2);
seven_dof = seven_dof.add_offset_to_joint_variable(6, -pi/2);

seven_dof = seven_dof.add_offset_to_joint_variable(4, pi/4);


seven_dof = seven_dof.set_joint_param(1, 2, 0);
seven_dof = seven_dof.set_joint_param(2, 2, 0.5);
seven_dof = seven_dof.set_joint_param(3, 2, 0.5);
seven_dof = seven_dof.set_joint_param(4, 2, 1);
seven_dof = seven_dof.set_joint_param(5, 2, 1);
seven_dof = seven_dof.set_joint_param(6, 2, 0.0);
seven_dof = seven_dof.set_joint_param(7, 2, 0.0);

%% Robot Draws Square
%% Square Points

seven_dof = seven_dof.create_A_matrices;
seven_dof = seven_dof.base_to_joints_transformations();

side_len = 1;

sq_p1 = seven_dof.joint_pose_list(:,1,num_of_links)';
sq_p2 = sq_p1 + [ 0          0           side_len];
sq_p3 = sq_p2 + [ side_len   0           0       ];
sq_p4 = sq_p3 + [ 0          0          -side_len];

square_points = [  sq_p2;
                   sq_p3;
                   sq_p4;];

square_points = [square_points; sq_p1;];

%% Draw Square
num_polygon_sides = 4;

eef_curr_vel = [0; 0; 0; 0; 0; 0;];
t_diff = 50;

for side = 1:num_polygon_sides
    seven_dof.joint_pose_list(:,1,num_of_links)
    eef_vel_update = cubic_path(seven_dof.joint_pose_list(:,1,num_of_links)', square_points(side,:), t_diff); % give vd values for t_diff steps/ one side of a square
    
    for t = 1:t_diff
        jaco = AnalyticalJacobian(seven_dof.transform_from_base, is_revolute_joints);
        % Convert eef_vel from 1x3 cartesian eef to 6 x 1 [d(q); alpha(q);]
        % First three rows = linear vel of eef
        % Last three rows = angular orientation vel of eef (don't care for 3 dof robots. set to zeros)

        eef_curr_vel(1:3) = eef_vel_update(t,:)';
        % eef_vel_update contains n time steps of vel updates. Need to perform this iteratively
        joint_vel_update = pinv(jaco)*eef_curr_vel; 
    
        % Integrate joint variable vel's to joint pose values
        seven_dof.table(:,1) = seven_dof.table(:,1) + (joint_vel_update.*seven_dof.is_revolute_joints');
        seven_dof.table(:,2) = seven_dof.table(:,2) + (joint_vel_update.*~seven_dof.is_revolute_joints');
        
        seven_dof = seven_dof.draw_lines();
        
        seven_dof = seven_dof.create_A_matrices;
        seven_dof = seven_dof.base_to_joints_transformations();
    end
end

disp('hw 3, 7DoF mechanism, job done')

