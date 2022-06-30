function plotting(format, x_list, t_list)
    
    switch format
        case 'full'
            x = x_list{1};
            t = t_list{1};
            plot(x(:,1),  x(:,2), 'LineWidth', 2)
            axis padded
            grid
            fprintf("Total time: %.4f\n", t)

        case 'steps'
            x = x_list{2};
            t = t_list{2};
            x_size = size(x);
            num_steps = x_size(2);
            legend_list = cell(num_steps, 1);
            hold on
            for i = 1:num_steps
                plot(x{i}(:,1),  x{i}(:,2), 'LineWidth', 2)
                legend_list{i} = strcat("Step ", num2str(i));
                fprintf("Time after step %d: %.4f\n", i, t{i})
            end
            hold off
            axis padded
            grid
            legend(legend_list)

        case 'phases'
            x = x_list{3};
            t = t_list{3};
            x_size = size(x);
            num_steps = idivide(x_size(2),int16(3));
            hold on
            for i = 1:num_steps
                j = (i-1)*3 + 1;

                plot(x{j}(:,1),  x{j}(:,2), 'LineWidth', 2, 'Color', [0.7 0 0])
                fprintf("Time after step %d, hind-leg: %.4f\n", i, t{j})

                plot(x{j+1}(:,1),  x{j+1}(:,2), 'LineWidth', 2, 'Color', [0 0.7 0])
                fprintf("Time after step %d, both-leg: %.4f\n", i, t{j+1})

                plot(x{j+2}(:,1),  x{j+2}(:,2), 'LineWidth', 2, 'Color', [0 0 0.7])
                fprintf("Time after step %d, fore-leg: %.4f\n", i, t{j+2})
            end
            hold off
            axis padded
            grid
            legend('Hind-leg stance', 'Both-legs stance', 'Fore-leg stance')

    end