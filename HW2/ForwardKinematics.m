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
        
        % list of joints and their respective poses in robot arm. Passed into constructor
        % initially because need initial joint positions.
        joint_pose_list;

        % updates for revolute angular displacement and primatic displacement
        update_ang_displacement = pi/100;
        update_prismatic_displacement = 0.5;
        
        % A Matrices, each 2d A matrix is stacked ontop of each other
        % creating a 3d matrix
        A_matrices;
        
    end
    
    methods
        function obj = ForwardKinematics(num_links, joint_pose_list, is_revolute_joints, a, alpha)
            if nargin == 5
                obj.num_links = num_links;
                obj.table = zeros(obj.num_links, 4);
                obj.A_matrices = zeros(4, 4, obj.num_links);
                
                obj.is_revolute_joints = is_revolute_joints;
                obj.joint_pose_list = joint_pose_list;
                
                obj.table(:,3) = a(:);
                obj.table(:,4) = alpha(:);
            end
        end
        function obj = update_joint_variables(obj)
            obj.table(:,1) = mod(obj.table(:,1) + obj.is_revolute_joints * obj.update_ang_displacement, 2*pi);
            
            % TODO: make separate prismatic update function so that the length does not extend infinitely
            obj.table(:,2) = mod(obj.table(:,2) + ~obj.is_revolute_joints * obj.update_prismatic_displacement, 10);
        end
        function obj = set_joint_variable(obj, joint_num, val)
            if obj.is_revolute_joint(joint_num) == 1
                obj.table(joint_num, 1) = val;
            else
                obj.table(joint_num, 2) = val;
            end
        end
        function obj = create_A_matrices(obj)
            for i = 1:obj.num_links
                A = [cos(obj.table(i,1))  -sin(obj.table(i,1))*cos(obj.table(i,4))   sin(obj.table(i,1))*sin(obj.table(i,4))  obj.table(i,3)*cos(obj.table(i,1));
                     sin(obj.table(i,1))   cos(obj.table(i,1))*cos(obj.table(i,4))  -cos(obj.table(i,1))*sin(obj.table(i,4))  obj.table(i,3)*sin(obj.table(i,1));
                     0                     sin(obj.table(i,4))                       cos(obj.table(i,4))                      obj.table(i,2)                    ;
                     0                     0                                         0                                        1                                 ];
                obj.A_matrices(:,:,i) = A;
            end
        end
        function obj = base_to_joints_transformations(obj)
            transform_from_base = obj.A_matrices;
            for i = 2:obj.num_links
                transform_from_base(:,:,i) = transform_from_base(:,:,i-1)*obj.A_matrices(:,:,i);
            end
            vector_transforms = transform_from_base(:,4,:); % extracts last column vector of all base to link transforms
            obj.joint_pose_list = vector_transforms.'; % updates joint poses by overwriting with new joint positions
        end
        function obj = draw_lines(obj)
            % TODO: Check how to draw lines from each joint (i-1) to joint (i)
        end
    end
end
            
            
        