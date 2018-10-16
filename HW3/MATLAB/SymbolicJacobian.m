% %% Create Symbolic Representation of RRP A matrices to detemine the Jacobian
% 
% is_revolute_joints = [1 1 0];
% num_links = 3;
% 
% % creation of A matrices
% theta = sym('theta', [1 num_links]);
% alpha = sym('alpha', [1 num_links]);
% d = sym('d', [1 num_links]);
% a = sym('a', [1 num_links]);
% 
% % fill in constants for RRP
% theta(3) = sym(0);
% d(2) = sym(0);
% a(1:3) = sym(0);
% alpha(1:2) = sym(pi/2);
% alpha(3) = sym(0);
% 
% for i = 1:num_links
%     A = sym([cos(theta(i))  -sin(theta(i))*cos(alpha(i))    sin(theta(i))*sin(alpha(i))  a(i)*cos(theta(i));
%              sin(theta(i))   cos(theta(i))*cos(alpha(i))   -cos(theta(i))*sin(alpha(i))  a(i)*sin(theta(i));
%              0               sin(alpha(i))                  cos(alpha(i))                d(i)              ;
%              0               0                              0                            1                 ]);
%    A_matrices(:,:,i) = A;
% end
% 
% % creation of T matrices
% transforms = A_matrices;
% 
% for i = 2:num_links
%     transforms(:,:,i) = transforms(:,:,i-1)*A_matrices(:,:,i);
% end
% 
% 
% 
% % strip columns 3 and 4 of transform matrices to build jacobian matrix
% 
% % always have O_0 be 3 column vector of 0's
% % always have z_0 be 3 column vector of 0's
% 
% jacobian = sym(zeros(6,num_links));
% 
% 
% o_n = transforms(1:3,4,num_links);
% o_0 = [0 0 0]';
% z_0 = [0 0 1]';
% 
% if is_revolute_joints(1) == 1
%     column_0 = [cross(z_0, (o_n-o_0)); z_0];
% else
%     column_0 = [z_0; 0; 0; 0];
% end
% 
% jacobian(:,1) = column_0;
% 
% for i = 2:num_links
%     z_curr = transforms(1:3,3,i-1);
%     if is_revolute_joints(i) == 1
%         % column vector for revolute joint
%         o_curr = transforms(1:3,4,i-1);
%         column = [cross(z_curr,(o_n-o_curr)); z_curr]; 
%     else
%         % column vector for prismatic joint
%         column = [z_curr; 0; 0; 0];
%     end
%     % save columns into 6xn jacobian
%     jacobian(:,i) = column;
% end
% 
% jacobian

%% Create Symbolic Representation of RRR A matrices to detemine the Jacobian

is_revolute_joints = [1 1 1];
num_links = length(is_revolute_joints);

% creation of A matrices
theta = sym('theta', [1 num_links]);
alpha = sym('alpha', [1 num_links]);
d = sym('d', [1 num_links]);
a = sym('a', [1 num_links]);

% fill in constants for RRP
d(3) = sym(0);
a(1:3) = sym(0);
alpha(1:3) = sym(0);

for i = 1:num_links
    A = sym([cos(theta(i))  -sin(theta(i))*cos(alpha(i))    sin(theta(i))*sin(alpha(i))  a(i)*cos(theta(i));
             sin(theta(i))   cos(theta(i))*cos(alpha(i))   -cos(theta(i))*sin(alpha(i))  a(i)*sin(theta(i));
             0               sin(alpha(i))                  cos(alpha(i))                d(i)              ;
             0               0                              0                            1                 ]);
   A_matrices(:,:,i) = A;
end

% creation of T matrices
transforms = A_matrices;

for i = 2:num_links
    transforms(:,:,i) = transforms(:,:,i-1)*A_matrices(:,:,i);
end

% strip columns 3 and 4 of transform matrices to build jacobian matrix

% always have O_0 be 3 column vector of 0's
% always have z_0 be 3 column vector of 0's

jacobian = sym(zeros(6,num_links));


o_n = transforms(1:3,4,num_links);
o_0 = [0 0 0]';
z_0 = [0 0 1]';

if is_revolute_joints(1) == 1
    column_0 = [cross(z_0, (o_n-o_0)); z_0];
else
    column_0 = [z_0; 0; 0; 0];
end

jacobian(:,1) = column_0;

for i = 2:num_links
    z_curr = transforms(1:3,3,i-1);
    if is_revolute_joints(i) == 1
        % column vector for revolute joint
        o_curr = transforms(1:3,4,i-1);
        column = [cross(z_curr,(o_n-o_curr)); z_curr]; 
    else
        % column vector for prismatic joint
        column = [z_curr; 0; 0; 0];
    end
    % save columns into 6xn jacobian
    jacobian(:,i) = column;
end

jacobian
