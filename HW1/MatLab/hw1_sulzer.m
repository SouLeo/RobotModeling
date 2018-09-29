m = [0.9851 -0.0881 0.1476 ; 0.1476 0.8735 -0.4640; -0.0881 0.4788 0.8735];
disp('matlab function output')
axang = rotm2axang(m)

disp('book method output')
m_trace = trace(m);
theta = acos((m_trace - 1) / 2);
const = (1 / (2*sin(theta))) ;
k = const * [m(3,2)-m(2,3);m(1,3)-m(3,1);m(2,1)-m(1,2)] % comment out the const variable to get the same output as the builtin matlab function.
