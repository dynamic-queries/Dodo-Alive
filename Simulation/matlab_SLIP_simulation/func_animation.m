function func_animation(anim_format, x_phases, a_phases, update_frequency, xi_vals, l0)

    switch anim_format
        case 'single'
            % shorten phase array (for flight-stance phase switching)
            num_phases = length(x_phases);
            max_x = x_phases{end}(end,1);
            max_y = 0;
            for i = 1:num_phases
                max_y = max([max([x_phases{i}(:,2)]); max_y]);
            end
        
            % animation initialization (plot first point and save plot in "h")
            figure
            h = plot(x_phases{1}(1,1), x_phases{1}(1,2));
            axis([0 max_x+0.01 0 max_y+0.01])
            title('Animation')
        
            % animation loop
            for i = 1:num_phases
                x_current = x_phases{i};
                a_current = a_phases{i};
                phase_typ = mod(i,2);
                for j = 1:length(x_current)
                    if phase_typ
                        leglen_x = cos(a_current(j)) * l0;
                        leglen_y = sin(a_current(j)) * l0;
                        toe = [x_current(j,1)+leglen_x, x_current(j,2)-leglen_y];
                        color = [0 0 0.9];
                    else
                        toe = [xi_vals(idivide(i, int16(2))), 0];
                        color = [0 0 0.3];
                    end
                    leg_x = linspace(toe(1), x_current(j,1), 50);
                    leg_y = linspace(toe(2), x_current(j,2), 50);
                    set(h, 'XData', leg_x, 'YData', leg_y, 'Linewidth', 1.5, 'Color', color)
                    pause(update_frequency)
                end
            end

        case 'double'
            warning("Animation for double SLIP model not implemented.")

        otherwise
            disp("Skipping animation.")
    end   
    