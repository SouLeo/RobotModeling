classdef ForwardKinematics
    properties
        % num links in robot's kinematic chain
        num_links;
        
        % table = DHTable of size (num_links x 4) the 4 represent the DH Variables
        % theta, d, a, alpha
        table; 
        
        % vector denoting whether or not the kinematic chain joints are revolute (1)
        % or prismatic (0)
        is_revolute_joints;
        
        % list of joints and their respective poses in robot arm. 
        joint_pose_list;
        
        % list of all EEF locations
%         history_eef_pose = zeros(3,1);

        % updates for revolute angular displacement and primatic displacement
        update_ang_displacement = pi/100;
        update_prismatic_displacement = 0.15625;
        
        % A Matrices, each 2d A matrix is stacked ontop of each other
        % creating a 3d matrix
        A_matrices;
        
        % Transform Matrices Relative to Ground Frame
        transform_from_base;

        
    end
    
    methods
        function obj = ForwardKinematics(num_links, is_revolute_joints, a, alpha)
            % Constructor of ForwardKinemeatics Class. 
            if nargin == 4
                obj.num_links = num_links;
                obj.table = zeros(obj.num_links, 4);
                obj.A_matrices = zeros(4, 4, obj.num_links);
                
                obj.is_revolute_joints = is_revolute_joints;
                
                obj.table(:,3) = a(:);
                obj.table(:,4) = alpha(:);
            end
        end
        function obj = update_joint_variables(obj)
            % Perform this function in an update loop. It increments joint variables (indicated by the boolean
            % is_revolute_joints vector) at every step. To adjust the displacement for each update, see class
            % variables: update_ang_displacement (revolute) and update_prismatic_displacement (prismatic.)
            obj.table(:,1) = mod(obj.table(:,1) + obj.is_revolute_joints' * obj.update_ang_displacement, 2*pi);
            
            % TODO: make separate prismatic update function so that the length does not extend infinitely
            % obj.table(:,2) = obj.table(:,2) + ~obj.is_revolute_joints' * obj.update_prismatic_displacement;

%             d = obj.table(:,2) .* ~obj.is_revolute_joints';
%             
%             d_out_of_bounds = d > 5;
%             obj.table(:,2) = obj.table(:,2) - d_out_of_bounds * obj.update_prismatic_displacement;
%             
%             d_in_bounds = d < 5;
%             obj.table(:,2) = obj.table(:,2) + d_in_bounds * obj.update_prismatic_displacement;
            obj.table(:,2) = mod(obj.table(:,2) + ~obj.is_revolute_joints' * obj.update_prismatic_displacement, 10);

        end
%         function obj = update_inv_joint_variables(obj, update_vel_vals)
%             % Overwrites a DH table joint variable with a user specified joint number and value.
%             q_vals = update_vel_vals(1:end-length(obj.is_revolute_joints));
%             obj.table(:, 1) = q_vals.*obj.is_revolute_joints;  % revolute joints update
%             obj.table(:, 2) = q_vals.*~obj.is_revolute_joints; % prismatic joints update
%         end
        function obj = set_joint_variable(obj, joint_num, val)
            % Overwrites a DH table joint variable with a user specified joint number and value.
            if obj.is_revolute_joints(joint_num) == 1
                % joint is revolute
                obj.table(joint_num, 1) = val;
            else
                % joint is prismatic
                obj.table(joint_num, 2) = val;
            end
        end
        function obj = set_joint_param(obj, joint_num, dh_param, val)
            % Sets a dh_param to a user specified value. dh_param is either 1, 2, 3, or 4.
            % these numbers correspond to theta, d, a, and alpha respectively.
            obj.table(joint_num, dh_param) = val;
        end
        function obj = add_offset_to_joint_variable(obj, joint_num, off)
            % Adds an offset to an existing DH table joint variable. The user specifies the joint
            % number and the offset.
            if obj.is_revolute_joints(joint_num) == 1
                % joint is revolute
                obj.table(joint_num, 1) = off + obj.table(joint_num, 1);
            else
                % joint is prismatic
                obj.table(joint_num, 2) = off + obj.table(joint_num, 2);
            end
        end
        function obj = create_A_matrices(obj)
            % Performs the A matrix math for SO3 spaces. It provides a 3D matrix of stacked A transformations.
            for i = 1:obj.num_links
                A = [cos(obj.table(i,1))  -sin(obj.table(i,1))*cos(obj.table(i,4))   sin(obj.table(i,1))*sin(obj.table(i,4))  obj.table(i,3)*cos(obj.table(i,1));
                     sin(obj.table(i,1))   cos(obj.table(i,1))*cos(obj.table(i,4))  -cos(obj.table(i,1))*sin(obj.table(i,4))  obj.table(i,3)*sin(obj.table(i,1));
                     0                     sin(obj.table(i,4))                       cos(obj.table(i,4))                      obj.table(i,2)                    ;
                     0                     0                                         0                                        1                                 ];
                obj.A_matrices(:,:,i) = A;
            end
        end
        function obj = base_to_joints_transformations(obj)
            obj.transform_from_base = obj.A_matrices;
            for i = 2:obj.num_links
                obj.transform_from_base(:,:,i) = obj.transform_from_base(:,:,i-1)*obj.A_matrices(:,:,i);
            end
            vector_transforms = obj.transform_from_base(:,4,:); % extracts last column vector of all base to link transforms
            vector_transforms(end,:,:) = []; % remove last row of matrix
            obj.joint_pose_list = vector_transforms; % updates joint poses by overwriting with new joint positions

        end
        function obj = draw_lines(obj)
            x = obj.joint_pose_list(1,1,:);
            y = obj.joint_pose_list(2,1,:);
            z = obj.joint_pose_list(3,1,:);
            
            size = obj.num_links;

            x = [zeros(1,1), reshape(x, [1, size])];
            y = [zeros(1,1), reshape(y, [1, size])];
            z = [zeros(1,1), reshape(z, [1, size])];
            
            scatter3(x, y, z, 200, 'red', 'filled')
            line(x, y, z, 'color', 'black', 'linewidth', 7)
          % 7DoF Demo 
            side_len = 1;
            
%             sq_p1 = [-1.1 1.2778 0.1364];
            sq_p1 = [-1 1.2071 0.2071];
            sq_p2 = sq_p1 + [ 0          0           side_len];
            sq_p3 = sq_p2 + [ side_len   0           0       ];
            sq_p4 = sq_p3 + [ 0          0          -side_len];
            
            square_points = [  sq_p2;
                               sq_p3;
                               sq_p4;];
            
            square_points = [square_points; sq_p1; sq_p2];
%             % RRP Demo
%             square_end_point = 2;
%             
%             square_points = [  square_end_point  0.00               square_end_point;
%                                square_end_point  square_end_point   square_end_point;
%                                square_end_point  square_end_point   0.00;
%                                square_end_point  0.00               0.00; 
%                                square_end_point  0.00               square_end_point;];
%             % RRR Demo
%             square_end_point = 1.00;
% 
%             square_points = [  square_end_point+1  square_end_point+1   0.00;
%                                1.00                square_end_point+1   0.00;
%                                1.00                1.00                 0.00;
%                                square_end_point+1  1.00                 0.00;
%                                square_end_point+1  square_end_point+1   0.00; ];
            line(square_points(:,1),square_points(:,2),square_points(:,3), 'color', 'blue', 'linewidth', 3)

            scale = 2;
            axis([-scale scale -scale scale -scale scale])
            drawnow    
        end
%         function obj = draw_square(obj)
%             square_points = [ 2   0   2;
%                               2   2   2;
%                               2   2   0;
%                               2   0   0;];
%             line(square_points(:,1),square_points(:,2),square_points(:,3), 'color', 'blue', 'linewidth', 3)
%         end
    end
end
            
            
        