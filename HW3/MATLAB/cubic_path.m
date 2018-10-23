function vel = cubic_path(cart_init, cart_final)
    t_init = 0;
    t_final = 4;
    v_init = [0 0 0];
    v_final = [0 0 0];
    t = linspace(t_init, t_final, 100*(t_final-t_init));

    c = ones(size(t));

    M = [1 t_init    t_init^2     t_init^3;
         0 1         2*t_init     3*t_init^2;
         1 t_final   t_final^2    t_final^3;
         0 1         2*t_final^2  3*t_final^2];
    b = [cart_init; v_init; cart_final; v_final];
    a = inv(M)*b;
    
    % qd = reference position trajectory
    % vd = reference velocity trajectory
    % ad = reference acceleration trajectory
 
    %qd = a(1,:).*c' + a(2,:).*t' + a(3,:).*t'.^2 + a(4,:).*t'.^3;
    
    vd = a(2,:).*c'+ 2*a(3,:).*t' + 3*a(4,:).*t'.^2;
    
    %const_start = cart_init.*c';
    %vels_at_time_steps = vd(:,2).*t';
    
    %integration_check = cumsum(vd(:,2));
    % ad = 2*a(3).*c + 6*a(4).*t;
    
    vel = vd;
end