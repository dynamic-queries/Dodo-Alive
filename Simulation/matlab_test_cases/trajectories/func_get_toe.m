function toe_trajectory = func_get_toe(x_phases, length_links, mass_links, mass_toe, mass_body, init_angles)

num_phases = length(x_phases);
traj = {num_phases};

l = length_links;
ml = mass_links;
mt = mass_toe;
mb = mass_body;
q1_0 = init_angles(1);
q2_0 = init_angles(2);
q3_0 = init_angles(2) - 2*pi;
flight_v = sin(q1_0)*l + sin(q1_0+q2_0)*l + sin(q1_0+q2_0+q3_0)*l;

% get CoM offset while resting length
denom = 3*mt + mt + mb;
flight_offset = (...
    sin(q1_0)*(l/2)*ml + ...
    (sin(q1_0)*l+sin(q1_0+q2_0)*(l/2))*ml + ...
    (sin(q1_0)*l+sin(q1_0+q2_0)*l+sin(q1_0+q2_0+q3_0)*(l/2))*ml + ...
    (sin(q1_0)*l+sin(q1_0+q2_0)*l+sin(q1_0+q2_0+q3_0)*l)*mt ...
    ) / denom;

assert(flight_v > flight_offset)
d0 = flight_v - flight_offset;

for p = 1:num_phases

    phase_type = mod(p,2);      % 1 flight, 0 stance
    com_traj = x_phases{p};

    if phase_type
        traj{p} = com_traj(:,1) - d0; %broadcasting?
    else
        traj{p} = 0; %broadcasting?
    end