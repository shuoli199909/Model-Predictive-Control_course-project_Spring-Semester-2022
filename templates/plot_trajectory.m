%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2022, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [fig_time,axes_time,fig_pos,axes_pos] = plot_trajectory(x,u,u_info,params)
    fig_time = figure;
    
    % check if input is 3-dimensional
    n_traj = size(x,3);
    assert(size(u,3) == n_traj);
    assert(size(u_info,3) == n_traj);

    nx = params.model.nx;
    assert(nx == size(x,1));

    t = 0:params.model.TimeStep:params.model.TimeStep*params.model.HorizonLength;

    % plot
    axes_time = cell(5,1);
    axes_time{1} = subplot(5,1,1);
    hold on;

    for i = 1:n_traj
        plot(axes_time{1},t,x(1,:,i),'DisplayName',sprintf('x_%d',i));
        plot(axes_time{1},t,x(3,:,i),'DisplayName',sprintf('z_%d',i));
    end
    legend('Location','EastOutside')

    % max position
    x_max = params.constraints.MaxAbsPositionXZ;
    plot(axes_time{1}, [t(1); t(end)],[x_max; x_max],'k--','HandleVisibility','off');
    plot(axes_time{1}, [t(1); t(end)],[-x_max; -x_max],'k--','HandleVisibility','off');
    ylabel('Position [Mm]')

    axes_time{2} = subplot(5,1,2);
    hold on;
    for i = 1:n_traj
        plot(axes_time{2},t,x(2,:,i),'DisplayName',sprintf('y_%d',i));
    end

    % max position
    y_max = params.constraints.MaxAbsPositionY;
    plot(axes_time{2},[t(1); t(end)],[y_max; y_max],'k--','HandleVisibility','off');
    plot(axes_time{2},[t(1); t(end)],[-y_max; -y_max],'k--','HandleVisibility','off');

    legend('Location','EastOutside')
    ylabel('Position [Mm]')

    axes_time{3} = subplot(5,1,3);
    hold on;
    for i = 1:n_traj
        plot(axes_time{3},t,x(4,:,i),'DisplayName',sprintf('v_{x%d}',i));
        plot(axes_time{3},t,x(5,:,i),'DisplayName',sprintf('v_{y%d}',i));
        plot(axes_time{3},t,x(6,:,i),'DisplayName',sprintf('v_{z%d}',i));
    end

    legend('Location','EastOutside')
    ylabel('Velocity [km/s]')

    axes_time{4} = subplot(5,1,4);
    hold on;
    % append one input value for plotting
    u(:,length(t),:) = u(:,length(t)-1,:);
    for i = 1:n_traj
        stairs(axes_time{4},t,u(1,:,i)','DisplayName',sprintf('u_{x%d}',i));
        stairs(axes_time{4},t,u(2,:,i)','DisplayName',sprintf('u_{y%d}',i));
        stairs(axes_time{4},t,u(3,:,i)','DisplayName',sprintf('u_{z%d}',i));
    end

    % max thrust
    thrust_max = params.constraints.MaxAbsThrust;
    plot([t(1); t(end)],[thrust_max; thrust_max],'k--','HandleVisibility','off');
    plot([t(1); t(end)],[-thrust_max; -thrust_max],'k--','HandleVisibility','off');
    legend('Location','EastOutside')
    ylabel('Thrust [N]')

    % feasibility
    axes_time{5} = subplot(5,1,5);
    hold on;
    for i = 1:n_traj
        % append one input value for plotting
        ctrl_feas = [u_info(:,:,i).ctrl_feas];
        ctrl_feas(length(t)) = ctrl_feas(length(t) - 1);
        stairs(axes_time{5},t,ctrl_feas','DisplayName',sprintf('feas_%d',i));
    end
    legend('Location','EastOutside')
    ylabel('Controller feasible [0/1]')

    % link axes
    axes_time = [axes_time{:}];
    linkaxes(axes_time,'x');
    xlabel('Time [s]')

    % plot without time
    % YX-plane
    fig_pos = figure;
    axes_pos = cell(2,1);
    axes_pos{1} = subplot(2,1,1);
    hold on
    colormap('hsv')

    for i = 1:n_traj
        plot(axes_pos{1},x(2,:,i),x(1,:,i));
    end

    axis equal
    xlabel('Y-Position [m]')
    ylabel('X-Position [Mm]')

    % YZ plane
    axes_pos{2} = subplot(2,1,2);
    hold on
    for i = 1:n_traj
        plot(axes_pos{2},x(2,:,i),x(3,:,i))
    end
    axis equal
    xlabel('Y-Position [Mm]')
    ylabel('Z-Position [Mm]')

    axes_pos = [axes_pos{:}];
    linkaxes(axes_pos,'x');
end