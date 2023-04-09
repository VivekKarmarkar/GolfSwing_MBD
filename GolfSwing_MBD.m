% Visualization flags
animation_flag = false;

% Physical constants
TH = 250;
alpha0 = deg2rad(-124);
mc = 0.394;
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
tf = 170 * 10^(-3);

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

% Initial conditions
y0 = [ic.theta0; ic.thetadot0; ic.psi0; ic.psidot0];

% ODE solver
opts = odeset('Mass',@(t,q) mass(t,q,params));
[t,q] = ode45(@(t,q) f(t,q,params), tspan, y0, opts);

% Initialize Impact metrics
drel = ones(length(t), 1);
clubspeed = ones(length(t), 1);

% Visualize Swing with Plot
figure(WindowState="maximized");
title('Golf Swing motion, Solved by ODE45');
hold on
for j = 1:length(t)
   theta = q(j,1);
   psi = q(j,3);
   xvals = [0 0 + La*cos(theta)];
   yvals = [0 0 + La*sin(theta)];
   xvals2 = [La*cos(theta) La*cos(theta) + lc*cos(psi)];
   yvals2 = [La*sin(theta) La*sin(theta) + lc*sin(psi)];
   drel(j, 1) = (xvals2(2) - ball.x0)^2 + (yvals2(2) - ball.y0)^2;
   if ~mod(j, resolution_swingvisual)
       plot(xvals,yvals,'k',xvals(1),yvals(1),'mo',xvals(2),yvals(2),'ro');
       plot(xvals2,yvals2,'k',xvals2(1),yvals2(1),'ro',xvals2(2),yvals2(2),'bo');
   end
   thetadot = q(j,2);
   psidot = q(j,4);
   clubspeed_sq = (La*thetadot)^2 + (Lc*psidot)^2 + 2*La*Lc*cos(psi-theta)*psidot*thetadot;
   clubspeed(j, 1) = sqrt(clubspeed_sq);
end
plot(ball.x0, ball.y0, '.b', 'MarkerSize', 35)
xlabel('x (m)');
ylabel('y (m)');
hold off
axis equal
grid
saveas(gcf,'GolfSwingPlot.png');
close(gcf);

% Plots for Time-domain Analysis
figure(WindowState="maximized");
sgtitle('Time-domain Analysis');

subplot(3,1,1)
plot(t, rad2deg(q(:,1)), 'r', 'LineWidth', 1);
xlabel('time (s)');
ylabel('\theta (degrees)')
title('Arm motion \theta');
grid;

subplot(3,1,2)
plot(t, rad2deg(q(:,3)), 'r', 'LineWidth', 1);
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

saveas(gcf,'TimeDomainPlot.png');
close(gcf);

% Plots for Impact Analysis
figure(WindowState="maximized");
sgtitle('Impact Analysis');

subplot(2,1,1)
plot(drel, rad2deg(q(:,3)), 'r', 'LineWidth', 1);
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

saveas(gcf,'ImpactPlot.png');
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

saveas(gcf,'TorquePlot.png');
close(gcf);

% Visualize Swing with Animation
if animation_flag
    vidObj = VideoWriter('golf_swing_animation.mp4', 'MPEG-4');
    open(vidObj);
    figure('WindowState', 'maximized');
    for j = 1:length(t)
       plot(ball.x0, ball.y0, '.b', 'MarkerSize', 35);
       xlim([-1.6, 0.6]);
       ylim([-1.5, 1]);
       grid;
       hold on;
    
       theta = q(j,1);
       psi = q(j,3);
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