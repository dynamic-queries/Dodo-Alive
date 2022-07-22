function inverse_kinematics = get_inverse(num_jl, len_links, forward_kin)

l1 = len_links(1);
l2 = len_links(2);

switch num_jl

    case 2
        % derive Jacobian (static matrix) of toe wrt to base-frame
        J = @(q1,q2) ...
            [-sin(q1)*l1-sin(q1+q2)*l2 -sin(q1+q2)*l2;
              cos(q1)*l1+cos(q1+q2)*l2  cos(q1+q2)*l2];

        % use Newton's Method to derive angles iteratively
        q = @(p, q_old) ...
            func_newton_method_inverse(p, q_old, J, forward_kin{3}, num_jl);

        inverse_kinematics = q;


    case 3 
        l3 = len_links(3);

        % derive Jacobian (static matrix) of toe wrt to base-frame
        % q3 = 2*pi - q2
        J = @(q1,q2) ...
            [-sin(q1)*l1-sin(q1+q2)*l2-sin(q1+2*pi)*l3 -sin(q1+q2)*l2;
              cos(q1)*l1+cos(q1+q2)*l2+cos(q1+2*pi)*l3  cos(q1+q2)*l2];

        % use Newton's Method to derive angles iteratively
        q = @(p, q_old) ...
            func_newton_method_inverse(p, q_old, J, forward_kin{4}, num_jl);

        inverse_kinematics = q;

    otherwise
        error("get_inverse() not implemented for joint number <2 and >3.")

end