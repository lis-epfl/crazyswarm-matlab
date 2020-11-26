function plotting(t_history, state_history, cmd_history)
%PLOTTING Plotting function for debugging purposes.
    
    t_history = t_history - t_history(1);

    figure(1)
    subplot(3, 1, 1)
    hold on
    grid on
    plot(t_history, squeeze(state_history(1, :, :))')
    plot(t_history, squeeze(cmd_history(1, :, :))', '-.k')
    xlabel("t [s]")
    ylabel("x [m]")
    subplot(3, 1, 2)
    hold on
    grid on
    plot(t_history, squeeze(state_history(2, :, :))')
    plot(t_history, squeeze(cmd_history(2, :, :))', '-.k')
    xlabel("t [s]")
    ylabel("y [m]")
    subplot(3, 1, 3)
    hold on
    grid on
    plot(t_history, squeeze(state_history(3, :, :))')
    plot(t_history, squeeze(cmd_history(3, :, :))', '-.k')
    xlabel("t [s]")
    ylabel("z [m]")
    
    figure(2)
    subplot(3, 1, 1)
    hold on
    grid on
    plot(t_history, squeeze(state_history(4, :, :))')
    plot(t_history, squeeze(cmd_history(4, :, :))', '-.k')
    xlabel("t [s]")
    ylabel("v_x [m/s]")
    subplot(3, 1, 2)
    hold on
    grid on
    plot(t_history, squeeze(state_history(5, :, :))')
    plot(t_history, squeeze(cmd_history(5, :, :))', '-.k')
    xlabel("t [s]")
    ylabel("v_y [m/s]")
    subplot(3, 1, 3)
    hold on
    grid on
    plot(t_history, squeeze(state_history(6, :, :))')
    plot(t_history, squeeze(cmd_history(6, :, :))', '-.k')
    xlabel("t [s]")
    ylabel("v_z [m/s]")
    
end

