function forward_kinematics = get_forward(num_jl, len_links)

l1 = len_links(1);
l2 = len_links(2);

switch num_jl

    case 2
        % position (x,y) and rotation of frame {1} (hip) wrt base-frame
        p_01 = @(q1)[0; 0; q1];

        % position (x,y) and rotation of frame {2} (knee) wrt base-frame
        p_02 = @(q1,q2)[cos(q1)*l1; 
                        sin(q1)*l1; 
                        q1+q2];

        % position (x,y) and rotation of EE (toe) wrt base-frame
        p_03 = @(q1,q2)[cos(q1)*l1+cos(q1+q2)*l2; 
                        sin(q1)*l1+sin(q1+q2)*l2;
                        q1+q2];

        % return
        forward_kinematics = {p_01, p_02, p_03};


    case 3
        l3 = len_links(3);

        % position (x,y) and rotation of frame {1} (hip) wrt base-frame
        p_01 = @(q1)[0; 0; q1];

        % position (x,y) and rotation of frame {2} (knee) wrt base-frame
        p_02 = @(q1,q2)[cos(q1)*l1; 
                        sin(q1)*l1; 
                        q1+q2];

        % position (x,y) and rotation of frame {3} (ankle) wrt base-frame
        % q3 = 2*pi - q1
        p_03 = @(q1,q2)[cos(q1)*l1+cos(q1+q2)*l2;
                           sin(q1)*l1+sin(q1+q2)*l2;
                           q2+2*pi];

        % position (x,y) and rotation of EE (toe) wrt base-frame
        p_04 = @(q1,q2)[cos(q1)*l1+cos(q1+q2)*l2+cos(q2+2*pi)*l3;
                        sin(q1)*l1+sin(q1+q2)*l2+sin(q2+2*pi)*l3;
                        q2+2*pi];
        
        % return
        forward_kinematics = {p_01, p_02, p_03, p_04};


    otherwise
        error("get_forward() not implemented for joint number <2 and >3.")

end
