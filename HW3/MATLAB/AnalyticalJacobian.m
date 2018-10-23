% strip columns 3 and 4 of transform matrices to build jacobian matrix

% always have O_0 be 3 column vector of 0's
% always have z_0 be 3 column vector of 0's

function jaco = AnalyticalJacobian(base_transforms, is_revolute_joints)
[~, ~, num_of_A_multiplications] = size(base_transforms);

num_links = num_of_A_multiplications;

jacobian = zeros(6,num_links);

o_n = base_transforms(1:3,4,num_links);
o_0 = [0 0 0]';
z_0 = [0 0 1]';

if is_revolute_joints(1) == 1
    column_0 = [cross(z_0, (o_n-o_0)); z_0];
else
    column_0 = [z_0; 0; 0; 0];
end

jacobian(:,1) = column_0;

for i = 2:num_links
    z_curr = base_transforms(1:3,3,i-1);
    if is_revolute_joints(i) == 1
        % column vector for revolute joint
        o_curr = base_transforms(1:3,4,i-1);
        column = [cross(z_curr,(o_n-o_curr)); z_curr]; 
    else
        % column vector for prismatic joint
        column = [z_curr; 0; 0; 0];
    end
    % save columns into 6xn jacobian
    jacobian(:,i) = column;
end
jaco = jacobian;
end
