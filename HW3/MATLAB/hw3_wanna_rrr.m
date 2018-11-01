%%% Selma Wanna
%%% Homework #3
%%% Dr. James Sulzer

%% initialize the DH Table for RRR Mechanism

is_revolute_joints = [1 1 1];
num_of_links = length(is_revolute_joints);
a_vector = [2 2 0];
alpha_vector = [0 0 0];
%% Forward Kinematics Setup

RRR = ForwardKinematics(num_of_links, is_revolute_joints, a_vector, alpha_vector); % RRP robot mechanism
RRR = RRR.add_offset_to_joint_variable(2, pi/2);

%% Robot Draws Square
%% Square Points
% TODO specify end points of square w.r.t starting pose of robot
RRR = RRR.create_A_matrices;
RRR = RRR.base_to_joints_transformations();
square_end_point = 1.00;

square_points = [  square_end_point+1  square_end_point+1   0.00;
                   1.00                square_end_point+1   0.00;
                   1.00                1.00                 0.00;
                   square_end_point+1  1.00                 0.00;];
             
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
        jaco = AnalyticalJacobian(RRR.transform_from_base, is_revolute_joints);
        % Convert eef_vel from 1x3 cartesian eef to 6 x 1 [d(q); alpha(q);]
        % First three rows = linear vel of eef
        % Last three rows = angular orientation vel of eef (don't care for 3 dof robots. set to zeros)
        
        eef_curr_vel(1:3) = eef_vel_update(t,:)';
        
        % eef_vel_update contains n time steps of vel updates. Need to perform this iteratively
        joint_vel_update = pinv(jaco)*eef_curr_vel; 
    
        % Integrate joint variable vel's to joint pose values
        RRR.table(:,1) = RRR.table(:,1) + (joint_vel_update.*RRR.is_revolute_joints');
        RRR.table(:,2) = RRR.table(:,2) + (joint_vel_update.*~RRR.is_revolute_joints');

        RRR = RRR.draw_lines();
        
        RRR = RRR.create_A_matrices;
        RRR = RRR.base_to_joints_transformations();
    end
end


disp('hw 3, rrr mechanism, job done')

