function dynamics = get_dynamics(len_links, mass_links, com_links, mass_toe, spring_stiffness, q2_resting_length)

assert(mass_links(1) == mass_links(2))
assert(len_links(1) == len_links(2))
m = mass_links(1);
l = len_links(1);
t = mass_toe;
g = 9.81;
k = spring_stiffness;
q2_r = q2_resting_length;


% with the spring and toe
tau_1 = @(q1,q2,q1d,q2d,q1dd,q2dd)( ...
        (1.5*m*l^2 + 0.5*m*l^2*cos(q2)) * q1dd + ...
        (0.25*m*l^2 + 0.25*m*l^2*cos(q2)) * q2dd - ...
        0.5*m*l^2*q1d*q2d*sin(q2) - 0.25*m*l^2*q2d^2*sin(q2) + ...
        1.5*m*g*l*cos(q1) + 0.5*m*g*l*cos(q1+q2) + ...
        (2*t*l^2 + 2*t*l^2*cos(q2)) * q1dd + ...
        (t*l^2 + t*l^2*cos(q2)) * q2dd - ...
        2*t*l^2*q1d*q2d*sin(q2) - t*l^2*q2d^2*sin(q2) + ...
        t*g*l*cos(q1+q2) ...
        );

tau_2 = @(q1,q2,q1d,q2d,q1dd,q2dd)( ...
        (0.25*m*l^2 + 0.25*m*l^2*cos(q2)) * q1dd + ...
        (0.25*m*l^2) * q2dd + ...
        0.25*m*l^2*q1d^2*sin(q2) + 0.25*k*l^2*sin(q2-q2_r) + ...
        0.5*m*g*l*cos(q1+q2) + ...
        (t*l^2 + t*l^2*cos(q2)) * q1dd + ...
        (t*l^2) * q2dd + ...
        t*l^2*q1d^2*sin(q2) + ...
        t*g*l*cos(q1+q2) ...
        );

dynamics = {tau_1, tau_2};
    