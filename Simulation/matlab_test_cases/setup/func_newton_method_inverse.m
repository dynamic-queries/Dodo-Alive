function joint_angles = func_newton_method_inverse(p, q_old, J, forward_kin, num_jl)


switch num_jl

    case 2
        p = p(1:2);
        q = q_old;
        for i = 1:100
            q_input = num2cell(q);
            f       = forward_kin(q_input{:});
            e       = p - f(1:2);
            q       = q + J(q_input{:}) \ e;
            if norm(e) < 0.001
                break
            end
        end
        % q1 \in [180, 360]
        % q2 \in [  0, 180]
        if ((q(1) < pi) || (q(1) > 2*pi))
                error("q1 exceeded the allowed range.")
        elseif ((q(2) > pi) || (q(2) < 0.0))
            error("q2 exceeded the allowed range.")
        end

        joint_angles = q;


    case 3 
        q = q_old;
        for i = 1:100
            q_input = num2cell(q);
            f       = forward_kin(q_input{:});
            e       = p - f(1:2);
            q       = q + J(q_input{:}) \ e;
            if norm(e) < 0.01
                break
            end
        end
        % q1 \in [180, 360]
        % q2 \in [180, 360]
        % q3 \in [  0, 180]
        %if ((q(1) < pi) || (q(1) > 2*pi))
        %    error("q1 exceeded the allowed range.")
        %elseif ((q(2) < pi) || (q(2) > 2*pi))
        %    error("q2 exceeded the allowed range.")
        %end

        joint_angles = q;

    otherwise
        error("Something went wrong with inverse kinematics.")

end
