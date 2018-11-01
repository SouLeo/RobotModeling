%%% Selma Wanna
%%% Homework #3
%%% Dr. James Sulzer

%% initialize the DH Table for RRP Mechanism

is_revolute_joints = [1 1 0];
num_of_links = length(is_revolute_joints);
a_vector = [0 0 0];
alpha_vector = [pi/2 pi/2 0];
%% Forward Kinematics Setup

RRP = ForwardKinematics(num_of_links, is_revolute_joints, a_vector, alpha_vector); % RRP robot mechanism
RRP = RRP.add_offset_to_joint_variable(2, pi/2);
RRP = RRP.set_joint_param(1, 2, 2); % on the first link, add a constant d offset of 2
RRP = RRP.set_joint_param(3, 2, 2); % on the third link, add a constant d offset of 2

%% Robot Draws Square
%% Square Points
% TODO specify end points of square w.r.t starting pose of robot
RRP = RRP.create_A_matrices;
RRP = RRP.base_to_joints_transformations();
square_end_point = 2+0.35; % +0.35 tolerance to compensate for 0 vel, slow down 

square_points = [  square_end_point  0.00               square_end_point;
                   square_end_point  square_end_point   square_end_point;
                   square_end_point  square_end_point   0.00;
                   square_end_point  0.00               0.00; ];
             
%% Draw Square
num_polygon_sides = 4;

eef_curr_vel = [0; 0; 0; 0; 0; 0;];

for side = 1:num_polygon_sides
    if side == num_polygon_sides
        end_point = mod(side+1, num_polygon_sides);
    else
        end_point = side + 1;
    end
    
    t_diff = 100;
    eef_vel_update = cubic_path(square_points(side,:), square_points(end_point,:), t_diff); % give vd values for 400 time steps/ one side of a square
    
    for t = 1:t_diff  
        jaco = AnalyticalJacobian(RRP.transform_from_base, is_revolute_joints);
        % Convert eef_vel from 1x3 cartesian eef to 6 x 1 [d(q); alpha(q);]
        % First three rows = linear vel of eef
        % Last three rows = angular orientation vel of eef (don't care for 3 dof robots. set to zeros)
        
        eef_curr_vel(1:3) = eef_vel_update(t,:)';
        
        % eef_vel_update contains n time steps of vel updates. Need to perform this iteratively
        joint_vel_update = pinv(jaco)*eef_curr_vel; 
    
        % Integrate joint variable vel's to joint pose values
        RRP.table(:,1) = RRP.table(:,1) + (joint_vel_update.*RRP.is_revolute_joints');
        RRP.table(:,2) = RRP.table(:,2) + (joint_vel_update.*~RRP.is_revolute_joints');

        RRP = RRP.draw_lines();
        
        RRP = RRP.create_A_matrices;
        RRP = RRP.base_to_joints_transformations();
    end
end


disp('hw 3, rrp mechanism, job done')

