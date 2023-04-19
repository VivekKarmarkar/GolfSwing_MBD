% Visualization flags
plot_flag = true;
animation_flag = false;
parametric_plot_flag = false;
parametric_analysis_plot_flag = true;
parametric_golf_swing_plot_flag = true;
comparative_golf_swing_plot_flag = true;

% Swing data struct
swing_data = struct;

parameter_range = [];

mc = 0.394;
dm = -0.005;

realistic_shot = true;
shot_idx = 0;
while realistic_shot

    % Physical constants
    TH = 250;
    alpha0 = deg2rad(-124);
    La = 0.615;
    Lc = 1.105;
    lc = 0.753;
    Ia = 1.15;
    Ic = 0.077;
    IA = Ia + mc*(La^2);
    IC = Ic + mc*(lc^2);
    MLS = mc * La * lc;
    k = IA + IC + 2*MLS*cos(alpha0);
    k0 = 0.5 * (IC - IA);
    k1 = (TH/k) * MLS * sin(alpha0);
    tf = 300 * 10^(-3);
    
    constants = struct;
    constants.La = La;
    constants.Lc = Lc;
    constants.lc = lc;
    
    % Ball struct
    ball = struct;
    ball.x0 = 0.19;
    ball.y0 = -1.35;
    
    % Create parameter struct
    params = struct;
    params.IA = IA;
    params.IC = IC;
    params.MLS = MLS;
    params.TH = TH;
    params.k = k;
    params.k0 = k0;
    params.k1 = k1;
    params.tf = tf;
    
    % Create initial conditions struct
    ic = struct;
    ic.theta0 = deg2rad(-166);
    ic.thetadot0 = 0;
    ic.psi0 = deg2rad(rad2deg(ic.theta0 + alpha0));
    ic.psidot0 = 0;
    
    % Create time vector
    resolution_simulation = 1000;
    resolution_swingvisual = 40;
    tspan = linspace(0,tf,resolution_simulation);
    tend_idx = length(tspan);
    
    % Initial conditions
    y0 = [ic.theta0; ic.thetadot0; ic.psi0; ic.psidot0];
    
    % ODE solver
    opts = odeset('Mass',@(t,q) mass(t,q,params));
    [t,q] = ode45(@(t,q) f(t,q,params), tspan, y0, opts);
    
    % Values at Impact
    [values_swing, values_impact] = compute_golf_swing(constants, ball, t, q);

    clubangle_condition = values_impact.clubangle > -90 && values_impact.clubangle < -80;
    clubspeed_condition = values_impact.clubspeed > 35;

    if clubangle_condition && clubspeed_condition
        shot_idx = shot_idx + 1;
        swing_data(shot_idx).mc = mc;
        swing_data(shot_idx).time = values_swing.time;
        swing_data(shot_idx).theta = values_swing.theta;
        swing_data(shot_idx).psi = values_swing.psi;
        swing_data(shot_idx).clubspeed = values_swing.clubspeed;
        swing_data(shot_idx).clubangle = values_swing.clubangle;
        swing_data(shot_idx).drel = values_swing.drel;
        swing_data(shot_idx).values_impact = values_impact;
        swing_data(shot_idx).constants = constants;
        swing_data(shot_idx).params = params;

        parameter_range = [parameter_range, mc];
        mc = mc + dm;
    else
        realistic_shot = false;
    end

end
numshots = shot_idx;
shots = 1:numshots;

% Initialize Impact metrics
drel = ones(length(t), 1);
clubspeed = ones(length(t), 1);

% Generate Plots
shot_idx_plot = 1;
generate_plots(plot_flag, swing_data, shot_idx_plot, ball, resolution_swingvisual);

% Generate Animation
shot_idx_animation = 1;
generate_animation(animation_flag, swing_data, shot_idx_animation, ball);

% Generate Parametric Plots
generate_parametric_plots(parametric_plot_flag, shots, swing_data);

% Generate Parametric Analysis Plots
generate_parametric_analysis_plots(parametric_analysis_plot_flag, numshots, swing_data, parameter_range);

% Generate Parametric Golf Swing Plots
generate_parametric_golf_swing_plots(parametric_golf_swing_plot_flag, shots, swing_data, ball);

% Generate Comparative Golf Swing Plots
generate_comparative_golf_swing_plots(comparative_golf_swing_plot_flag, shots, swing_data, ball);

% Function to generate golf swing comparative plots
function generate_comparative_golf_swing_plots(comparative_golf_swing_plot_flag, shots, swing_data, ball)

    if comparative_golf_swing_plot_flag

        figure(5);
        sgtitle('Golf Swing motion, Solved by ODE45');
        hold on;

        comparative_shot_idx_list = [shots(1), shots(end)];
        shot_LW = [2, 1];
        shot_CO = ['k', 'm'];
        shot_name = ["heavy club", "light club"];

        for s = 1:length(comparative_shot_idx_list)

            shot_idx = comparative_shot_idx_list(s);

            constants = swing_data(shot_idx).constants;
            La = constants.La;
            lc = constants.lc;

            values_theta = deg2rad(swing_data(shot_idx).theta);
            values_psi = deg2rad(swing_data(shot_idx).psi);

            resolution_swingvisual = 0.07;
            resolution_swingvisual_normalized = ceil(resolution_swingvisual * swing_data(shot_idx).values_impact.tend_idx);

            for j = flip(1:swing_data(shot_idx).values_impact.tend_idx)
               

               theta = values_theta(j,1);
               psi = values_psi(j,1);
               xvals = [0 0 + La*cos(theta)];
               yvals = [0 0 + La*sin(theta)];
               xvals2 = [La*cos(theta) La*cos(theta) + lc*cos(psi)];
               yvals2 = [La*sin(theta) La*sin(theta) + lc*sin(psi)];
 
               if ~mod(swing_data(shot_idx).values_impact.tend_idx-j, resolution_swingvisual_normalized)
                   
                   plot(xvals,yvals, shot_CO(s), 'LineWidth', shot_LW(s), 'HandleVisibility', 'off');
                   plot(xvals(1),yvals(1),'mo', xvals(2),yvals(2),'ro', 'HandleVisibility', 'off');
                   plot(xvals2,yvals2, shot_CO(s), 'LineWidth', shot_LW(s), 'HandleVisibility', 'off');
                   plot(xvals2(1),yvals2(1),'ro', xvals2(2),yvals2(2),'bo', 'HandleVisibility', 'off');

               end


               if j == swing_data(shot_idx).values_impact.tend_idx
                
                   plot(nan, shot_CO(s), 'LineWidth', shot_LW(s), 'DisplayName', shot_name(s) + " (m_{c} = " + num2str(swing_data(shot_idx).mc) + " kg)");

               end


            end

        end

        plot(nan, 'mo', 'DisplayName', 'shoulder joint');
        plot(nan, 'ro', 'DisplayName', 'wrist joint');
        plot(nan, 'bo', 'DisplayName', 'club head');

        plot(ball.x0, ball.y0, '.b', 'MarkerSize', 35, 'DisplayName', 'ball');
        xlabel('x (m)');
        ylabel('y (m)');
        hold off;
        axis equal;
        grid on;
        legend;

        ax5 = figure(5);
        ax5.WindowState="maximized";
        saveas(ax5, 'GolfSwingPlot_comparitive_2.png');
        close(ax5);

    end

end

% Function to generate golf swing parametric plots
function generate_parametric_golf_swing_plots(parametric_golf_swing_plot_flag, shots, swing_data, ball)

    if parametric_golf_swing_plot_flag

        figure(4);
        tiledlayout(2,2,"TileSpacing","tight");
        sgtitle('Golf Swing motion, Solved by ODE45');

        plot_idx = 1;

        for shot_idx = 1:3:length(shots)

            constants = swing_data(shot_idx).constants;
            La = constants.La;
            lc = constants.lc;

            values_theta = deg2rad(swing_data(shot_idx).theta);
            values_psi = deg2rad(swing_data(shot_idx).psi);

            resolution_swingvisual = 0.07;
            resolution_swingvisual_normalized = ceil(resolution_swingvisual * swing_data(shot_idx).values_impact.tend_idx);

            nexttile;
            title("m_{c} = " + num2str(swing_data(shot_idx).mc) + " kg");
            hold on;

            for j = flip(1:swing_data(shot_idx).values_impact.tend_idx)
               
               theta = values_theta(j,1);
               psi = values_psi(j,1);
               xvals = [0 0 + La*cos(theta)];
               yvals = [0 0 + La*sin(theta)];
               xvals2 = [La*cos(theta) La*cos(theta) + lc*cos(psi)];
               yvals2 = [La*sin(theta) La*sin(theta) + lc*sin(psi)];

               if ~mod(swing_data(shot_idx).values_impact.tend_idx-j, resolution_swingvisual_normalized)
                   plot(xvals,yvals,'k',xvals(1),yvals(1),'mo',xvals(2),yvals(2),'ro', 'HandleVisibility', 'off');
                   plot(xvals2,yvals2,'k',xvals2(1),yvals2(1),'ro',xvals2(2),yvals2(2),'bo', 'HandleVisibility', 'off');
               end

            end

            plot(ball.x0, ball.y0, '.b', 'MarkerSize', 35, 'DisplayName', 'ball');

            if shot_idx == 1
                plot(nan, 'mo', 'DisplayName', 'shoulder joint');
                plot(nan, 'ro', 'DisplayName', 'wrist joint');
                plot(nan, 'bo', 'DisplayName', 'club head');
                legend;
            end

            xlabel('x (m)');
            ylabel('y (m)');
            hold off;
            axis equal;
            grid on;

            plot_idx = plot_idx + 1;

        end

        ax4 = figure(4);
        ax4.WindowState="maximized";
        saveas(ax4, 'GolfSwingPlot_parametric_2.png');
        close(ax4);

    end

end

% Function to generate Parametric Analysis Plots
function generate_parametric_analysis_plots(parametric_analysis_plot_flag, numshots, swing_data, parameter_range)
    
    if parametric_analysis_plot_flag

        % Parametric Analysis for Impact metrics
        ti_list = nan(numshots, 1);
        cs_avg_list = nan(numshots, 1);
        csi_list = nan(numshots, 1);
        cai_list = nan(numshots, 1);
        for j=1:12
            ti_list(j, 1) = swing_data(j).values_impact.time;
            cs_avg_list(j, 1) = mean(swing_data(j).clubspeed);
            csi_list(j, 1) = swing_data(j).values_impact.clubspeed;
            cai_list(j, 1) = swing_data(j).values_impact.clubangle;
        end
        
        figure(WindowState="maximized");
        sgtitle('Parametric Analysis');
        
        subplot(2,2,1)
        plot(parameter_range, ti_list, 'b', 'LineWidth', 1);
        title('Downswing time vs Club mass (m_{c})');
        xlabel('m_{c} (kg)');
        ylabel('Downswing time (s)');
        grid on;
        
        subplot(2,2,2)
        plot(parameter_range, csi_list, 'k', 'LineWidth', 1);
        title('Impact Clubspeed vs Club mass (m_{c})');
        xlabel('m_{c} (kg)');
        ylabel('Impact Clubspeed (m/s)');
        grid on;

        subplot(2,2,3)
        plot(parameter_range, cai_list, 'r', 'LineWidth', 1);
        title('Impact Clubangle vs Club mass (m_{c})');
        xlabel('m_{c} (kg)');
        ylabel('\psi (degrees)');
        grid on;

        subplot(2,2,4)
        plot(parameter_range, cs_avg_list, 'r', 'LineWidth', 1);
        title('Average Clubspeed vs Club mass (m_{c})');
        xlabel('m_{c} (kg)');
        ylabel('Average Clubspeed (m/s)');
        grid on;
        
        saveas(gcf,'ParametricAnalysisPlot_parametric_2.png');
        close(gcf);

    end

end

% Function to generate animation
function generate_animation(animation_flag, swing_data, shot_idx, ball)
    
    constants = swing_data(shot_idx).constants;
    
    La = constants.La;
    lc = constants.lc;

    values_impact = swing_data(shot_idx).values_impact;

    values_theta = deg2rad(swing_data(shot_idx).theta);
    values_psi = deg2rad(swing_data(shot_idx).psi);
    
    % Visualize Swing with Animation
    if animation_flag
        vidObj = VideoWriter('golf_swing_animation_2.mp4', 'MPEG-4');
        open(vidObj);
        figure('WindowState', 'maximized');
        for j = 1:values_impact.tend_idx
           plot(ball.x0, ball.y0, '.b', 'MarkerSize', 35);
           xlim([-1.6, 0.6]);
           ylim([-1.5, 1]);
           grid;
           hold on;
        
           theta = values_theta(j,1);
           psi = values_psi(j,1);
           xvals = [0 0 + La*cos(theta)];
           yvals = [0 0 + La*sin(theta)];
           xvals2 = [La*cos(theta) La*cos(theta) + lc*cos(psi)];
           yvals2 = [La*sin(theta) La*sin(theta) + lc*sin(psi)];
        
           plot(xvals,yvals,'k',xvals(1),yvals(1),'mo',xvals(2),yvals(2),'ro');
           plot(xvals2,yvals2,'k',xvals2(1),yvals2(1),'ro',xvals2(2),yvals2(2),'bo');
           xlabel('x (m)');
           ylabel('y (m)');
           title('Golf Swing motion, Solved by ODE45');
           pause(0.5);
           frame = getframe(gcf);
           writeVideo(vidObj, frame);
           clf;
        end
        close(vidObj);
        close(gcf);
    end
end

% Function to generate all plots
function generate_plots(plot_flag, swing_data, shot_idx, ball, resolution_swingvisual)
    
    constants = swing_data(shot_idx).constants;
    La = constants.La;
    lc = constants.lc;

    params = swing_data(shot_idx).params;
    TH = params.TH;
    k = params.k;
    k0 = params.k0;
    k1 = params.k1;

    values_impact = swing_data(shot_idx).values_impact;

    values_theta = deg2rad(swing_data(shot_idx).theta);
    values_psi = deg2rad(swing_data(shot_idx).psi);

    clubspeed = swing_data(shot_idx).clubspeed;
    drel = swing_data(shot_idx).drel;
    t = swing_data(shot_idx).time;

    % Visualize Swing with Plot
    if plot_flag
        figure(WindowState="maximized");
        title('Golf Swing motion, Solved by ODE45');
        hold on
        for j = 1:values_impact.tend_idx
           theta = values_theta(j,1);
           psi = values_psi(j,1);
           xvals = [0 0 + La*cos(theta)];
           yvals = [0 0 + La*sin(theta)];
           xvals2 = [La*cos(theta) La*cos(theta) + lc*cos(psi)];
           yvals2 = [La*sin(theta) La*sin(theta) + lc*sin(psi)];
           drel(j, 1) = (xvals2(2) - ball.x0)^2 + (yvals2(2) - ball.y0)^2;
           if ~mod(j, resolution_swingvisual)
               plot(xvals,yvals,'k',xvals(1),yvals(1),'mo',xvals(2),yvals(2),'ro','HandleVisibility','off');
               plot(xvals2,yvals2,'k',xvals2(1),yvals2(1),'ro',xvals2(2),yvals2(2),'bo','HandleVisibility','off');
           end
        end
        plot(nan, 'mo', 'DisplayName', 'shoulder joint');
        plot(nan, 'ro', 'DisplayName', 'wrist joint');
        plot(nan, 'bo', 'DisplayName', 'club head');
        plot(ball.x0, ball.y0, '.b', 'MarkerSize', 35, 'DisplayName', 'ball')
        xlabel('x (m)');
        ylabel('y (m)');
        hold off
        axis equal
        grid
        legend
        saveas(gcf,'GolfSwingPlot_2.png');
        close(gcf);
    
        % Plots for Time-domain Analysis
        figure(WindowState="maximized");
        sgtitle('Time-domain Analysis');
        
        subplot(3,1,1)
        plot(t, rad2deg(values_theta), 'r', 'LineWidth', 1);
        xlabel('time (s)');
        ylabel('\theta (degrees)')
        title('Arm motion \theta');
        grid;
        
        subplot(3,1,2)
        plot(t, rad2deg(values_psi), 'r', 'LineWidth', 1);
        hold on;
        yline(-90, 'k--', 'LineWidth', 1);
        xlabel('time (s)');
        ylabel('\psi (degrees)');
        title('Club motion \psi');
        grid;
        
        subplot(3,1,3)
        plot(t, clubspeed, 'r', 'LineWidth', 1);
        xlabel('time (s)');
        ylabel('club speed (m/s)')
        title('Club speed (m/s)');
        grid;
        
        saveas(gcf,'TimeDomainPlot_2.png');
        close(gcf);
        
        % Plots for Impact Analysis
        figure(WindowState="maximized");
        sgtitle('Impact Analysis');
        
        subplot(2,1,1)
        plot(drel, rad2deg(values_psi), 'r', 'LineWidth', 1);
        set(gca, 'xdir', 'reverse');
        hold on;
        yline(-90, 'k--', 'LineWidth', 1);
        xlabel('distance between ball and club head (m)');
        ylabel('\psi (degrees)');
        title('Club motion relative to ball \psi');
        grid;
        
        subplot(2,1,2)
        plot(drel, clubspeed, 'r', 'LineWidth', 1);
        set(gca, 'xdir', 'reverse');
        hold on;
        xlabel('distance between ball and club head (m)');
        ylabel('club speed (m/s)');
        title('Club speed plot');
        grid;
        
        saveas(gcf,'ImpactPlot_2.png');
        close(gcf);
        
        % Torque Model
        torque_multiplier = 5;
        torque_offset = 20;
        TW_model = -((1 - heaviside(t-(k0/k1))) .* ((TH/k).*(k0 - k1*t)));
        TW_model = torque_offset + torque_multiplier * TW_model;
        TH_model = TH * ones(length(t), 1);
        
        % Plots for Torque Model
        figure(WindowState="maximized");
        sgtitle('Torque Model');
        
        subplot(2,1,1)
        plot(t, TH_model, 'b', 'LineWidth', 1);
        xlabel('time (s)');
        ylabel('TH (N-m)');
        title('Shoulder or Hub Torque (TH)');
        grid;
        
        subplot(2,1,2)
        plot(t, TW_model, 'r', 'LineWidth', 1);
        hold on;
        xline(k0/k1, 'k--');
        xlabel('time (s)');
        ylabel('TW (N-m)');
        title('Wrist Torque (TW)');
        grid;
        
        saveas(gcf,'TorquePlot_2.png');
        close(gcf);
    
    end
end

% Function to generate parametric plots
function generate_parametric_plots(parametric_plot_flag, shots, swing_data)

    if parametric_plot_flag

        for shot_idx = 1:3:length(shots)

            params = swing_data(shot_idx).params;
            TH = params.TH;
            k = params.k;
            k0 = params.k0;
            k1 = params.k1;
    
            t_normalized = swing_data(shot_idx).time / swing_data(shot_idx).values_impact.time;
            theta = swing_data(shot_idx).theta;
            psi = swing_data(shot_idx).psi;
            clubspeed = swing_data(shot_idx).clubspeed;
            drel = swing_data(shot_idx).drel;

            mc = swing_data(shot_idx).mc;

            figure(1);
            sgtitle('Time-domain Analysis');

            subplot(3,1,1)
            plot(t_normalized, theta, 'LineWidth', 1, 'DisplayName',"m_{c} = " + num2str(mc));
            hold on;
            xlabel('normalized time');
            ylabel('\theta (degrees)')
            title('Arm motion \theta');
            grid on;
            legend(Location="northwest");
            
            subplot(3,1,2)
            hold on;
            if shot_idx == 1
                yline(-90, 'k--', 'LineWidth', 1, 'DisplayName', "\psi = 90^{\circ}");
            end
            plot(t_normalized, psi, 'LineWidth', 1, 'DisplayName',"m_{c} = " + num2str(mc));
            xlabel('normalized time');
            ylabel('\psi (degrees)');
            title('Club motion \psi');
            grid on;
            legend(Location="northwest");
            
            subplot(3,1,3)
            plot(t_normalized, clubspeed, 'LineWidth', 1, 'DisplayName',"m_{c} = " + num2str(mc));
            hold on;
            xlabel('normalized time');
            ylabel('club speed (m/s)')
            title('Club speed (m/s)');
            grid on;
            legend(Location="northwest");

            figure(2);
            sgtitle('Impact Analysis');
        
            subplot(2,1,1)
            if shot_idx == 1
                yline(-90, 'k--', 'LineWidth', 1, 'DisplayName', "\psi = 90^{\circ}");
            end
            hold on;
            plot(drel, psi, 'LineWidth', 1, 'DisplayName',"m_{c} = " + num2str(mc));
            set(gca, 'xdir', 'reverse');
            xlabel('distance between ball and club head (m)');
            ylabel('\psi (degrees)');
            title('Club motion relative to ball \psi');
            grid on;
            legend(Location="northwest");
            
            subplot(2,1,2)
            plot(drel, clubspeed, 'LineWidth', 1, 'DisplayName',"m_{c} = " + num2str(mc));
            hold on;
            set(gca, 'xdir', 'reverse');
            xlabel('distance between ball and club head (m)');
            ylabel('club speed (m/s)');
            title('Club speed plot');
            grid on;
            legend(Location="northwest");

            torque_multiplier = 5;
            torque_offset = 20;
            TW_model = -((1 - heaviside(t_normalized-(k0/k1))) .* ((TH/k).*(k0 - k1*t_normalized)));
            TW_model = torque_offset + torque_multiplier * TW_model;
            TH_model = TH * ones(length(t_normalized), 1);

            figure(3);
            sgtitle('Torque Model');
        
            subplot(2,1,1)
            plot(t_normalized, TH_model, 'LineWidth', 1, 'DisplayName',"m_{c} = " + num2str(mc));
            hold on;
            xlabel('normalized time');
            ylabel('TH (N-m)');
            title('Shoulder or Hub Torque (TH)');
            grid on;
            legend;
            
            subplot(2,1,2)
            plot(t_normalized, TW_model, 'LineWidth', 1, 'DisplayName',"m_{c} = " + num2str(mc));
            hold on;
            xlabel('normalized time');
            ylabel('TW (N-m)');
            title('Wrist Torque (TW)');
            grid on;
            legend;

        end

        ax1 = figure(1);
        ax1.WindowState="maximized";
        saveas(ax1, 'TimeDomainPlot_parametric_2.png');
        close(ax1);
        
        ax2 = figure(2);
        ax2.WindowState="maximized";
        saveas(ax2, 'ImpactPlot_parametric_2.png');
        close(ax2);

        ax3 = figure(3);
        ax3.WindowState="maximized";
        saveas(ax3, 'TorquePlot_parametric_2.png');
        close(ax3);
    
    end
end

% Generate values at impact
function [values_swing, values_impact] = compute_golf_swing(constants, ball, t, q)

    values_swing = struct;
    values_impact = struct;

    drel = nan(length(t), 1);
    clubspeed = nan(length(t), 1);

    La = constants.La;
    Lc = constants.Lc;
    lc = constants.lc;

    for j = 1:length(t)
       
       theta = q(j,1);
       psi = q(j,3);
       xvals2 = [La*cos(theta) La*cos(theta) + lc*cos(psi)];
       yvals2 = [La*sin(theta) La*sin(theta) + lc*sin(psi)];
       drel(j, 1) = (xvals2(2) - ball.x0)^2 + (yvals2(2) - ball.y0)^2;
       thetadot = q(j,2);
       psidot = q(j,4);
       clubspeed_sq = (La*thetadot)^2 + (Lc*psidot)^2 + 2*La*Lc*cos(psi-theta)*psidot*thetadot;
       clubspeed(j,1) = sqrt(clubspeed_sq);

       if xvals2(2) > ball.x0
           
           disp("Downswing completed");
           tend_idx = j-1;

           values_impact.tend_idx = j-1;
           values_impact.time = t(tend_idx, 1);
           values_impact.clubspeed = sqrt(clubspeed_sq);
           values_impact.clubangle = rad2deg(q(tend_idx, 3));

           values_swing.time = t(1:tend_idx, 1);
           values_swing.theta = rad2deg(q(1:tend_idx, 1));
           values_swing.psi = rad2deg(q(1:tend_idx, 3));
           values_swing.clubspeed = clubspeed(1:tend_idx, 1);
           values_swing.clubangle = rad2deg(q(1:tend_idx, 3));
           values_swing.drel = drel(1:tend_idx, 1);

           break;

       end

    end
    
end

% Equation to solve
function dydt = f(t,q,P)
    % Extract parameters
    MLS = P.MLS;
    TH = P.TH;
    k = P.k;
    k0 = P.k0;
    k1 = P.k1;

    torque_multiplier = 5;
    torque_offset = 20;
    
    TW = -((1 - heaviside(t-(k0/k1))) .* ((TH/k).*(k0 - k1*t)));
    TW = torque_offset + torque_multiplier * TW;

    % RHS of system of equations
    dydt = [q(2)
            MLS*q(4)^2*sin(q(3)-q(1)) + TH - TW
            q(4)
            -MLS*q(2)^2*sin(q(3)-q(1)) + TW];
end

% Mass matrix function
function M = mass(~,q,P)
    % Extract parameters
    IA = P.IA;
    IC = P.IC;
    MLS = P.MLS;
    
    % Set nonzero elements in mass matrix
    M = zeros(4,4);
    M(1,1) = 1;
    M(2,2) = IA;
    M(2,4) = MLS*cos(q(3)-q(1));
    M(3,3) = 1;
    M(4,2) = MLS*cos(q(3)-q(1));
    M(4,4) = IC;
end