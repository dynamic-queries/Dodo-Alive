function func_plotting(format, x_list, t_list, over_time, vertical)
    
    if vertical
        offset = 1.0;
    else
        offset = 0.01;
    end
    
    hold on
    switch format
        case 'full'
            x = x_list{1};
            t = t_list{1};
            if over_time 
                s = t;
            else 
                s = x;
            end
            plot(s(:,1),  x(:,2), 'LineWidth', 2)
            axis([0 max(s(:,1))+offset 0 max(x(:,2))+0.01])
            title('Trajectory plot (full)')
            grid
            hold off
            fprintf("Total time: %.4f\n", t(end))

        case 'steps'
            x = x_list{2};
            t = t_list{2};
            if over_time 
                s = t;
            else 
                s = x;
            end
            x_size = size(x);
            num_steps = x_size(2);
            legend_list = cell(num_steps, 1);
            max_y = 0;
            for i = 1:num_steps
                plot(s{i}(:,1),  x{i}(:,2), 'LineWidth', 2)
                legend_list{i} = strcat("Step ", num2str(i));
                max_y = max([max([x{i}(:,2)]); max_y]);
                fprintf("Time after step %d: %.4f\n", i, t{i}(end))
            end
            axis([0 max(s{num_steps}(:,1))+offset 0 max_y+0.01])
            title('Trajectory plot (steps)')
            grid
            hold off
            legend(legend_list)

        case 'phases'
            x = x_list{3};
            t = t_list{3};
            if over_time 
                s = t;
            else 
                s = x;
            end
            x_size = size(x);
            num_phases = x_size(2);
            max_y = 0;
            for i = 1:num_phases
                phase_typ = mod(i,2);
                if phase_typ
                    plot(s{i}(:,1),  x{i}(:,2), 'LineWidth', 2, 'Color', [0 0 0.7])
                    max_y = max([max([x{i}(:,2)]); max_y]);
                    fprintf("Time after phase %d (flight): %.4f\n", i, t{i}(end))
                else
                    plot(s{i}(:,1),  x{i}(:,2), 'LineWidth', 2, 'Color', [0.7 0 0])
                    max_y = max([max([x{i}(:,2)]); max_y]);
                    fprintf("Time after phase %d (stance): %.4f\n", i, t{i}(end))
                end
            end
            axis([0 max(s{num_phases}(:,1))+offset 0 max_y+0.01])
            title('Trajectory plot (phases)')
            grid
            hold off
            legend('Flight phase', 'Stance phase')

        case 'detail'
            x = x_list{4};
            t = t_list{4};
            if over_time 
                s = t;
            else 
                s = x;
            end
            x_size = size(x);
            num_steps = idivide(x_size(2),int16(3));
            max_y = 0;
            for i = 1:num_steps
                j = (i-1)*3 + 1;

                plot(s{j}(:,1),  x{j}(:,2), 'LineWidth', 2, 'Color', [0 0 0.7])
                max_y = max([max([x{j}(:,2)]); max_y]);
                fprintf("Time after step %d, apex->TD: %.4f\n", i, t{j}(end))

                plot(s{j+1}(:,1),  x{j+1}(:,2), 'LineWidth', 2, 'Color', [0.7 0 0])
                max_y = max([max([x{j+1}(:,2)]); max_y]);
                fprintf("Time after step %d, stance  : %.4f\n", i, t{j+1}(end))

                plot(s{j+2}(:,1),  x{j+2}(:,2), 'LineWidth', 2, 'Color', [0 0.7 0])
                max_y = max([max([x{j+2}(:,2)]); max_y]);
                fprintf("Time after step %d, LO->apex: %.4f\n", i, t{j+2}(end))
            end
            axis([0 max(s{num_steps*3}(:,1))+offset 0 max_y+0.01])
            title('Trajectory plot (detailed)')
            grid
            legend('Flight phase (from apex)', 'Stance phase', 'Flight phase (to apex)')

        otherwise
            disp("Skipping plotting.")

    end