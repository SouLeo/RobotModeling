function vel = cubic_path(cart_init, cart_final, delta_t)
    t_diff = delta_t;
    v_init = [0 0 0];
    v_final = [0 0 0];
    t = linspace(0, t_diff, t_diff);

    c = ones(size(t));

    M = [1      0          0            0;
         0      1          0            0;
         1      t_diff     t_diff^2     t_diff^3;
         0      1          2*t_diff^2   3*t_diff^2];
    b = [cart_init; v_init; cart_final; v_final];
    a = inv(M)*b;
    
    % qd = reference position trajectory
    % qd = a(1,:).*c' + a(2,:).*t' + a(3,:).*t'.^2 + a(4,:).*t'.^3;
    % vd = reference velocity trajectory
    vel = a(2,:).*c'+ 2*a(3,:).*t' + 3*a(4,:).*t'.^2;
    % ad = reference acceleration trajectory
    % ad = 2*a(3).*c + 6*a(4).*t;

end