%% \Ddot{x} = \tau + d (acceleration level control)

clc;
clear;

%% Load Data of VCZ
poses_target_obs = readmatrix("poses.csv");

VCZ_rob1 = poses_target_obs(1:9,1:2);
VCZ_rob2 = poses_target_obs(1:9,3:4);
f_targets = poses_target_obs(10:11,:);
f_obstacles = poses_target_obs(12:13,:);

% Interpolate and add more data
VCZ_interp_rob1 = [];
VCZ_interp_rob2 = [];

v = 0.1;    % constant velocity
dt = 0.001;  % time step

for i=1:size(VCZ_rob1, 1)-1
    len1 = norm(VCZ_rob1(i,:)-VCZ_rob1(i+1,:));
    len2 = norm(VCZ_rob2(i,:)-VCZ_rob2(i+1,:));

    N1 = round(len1/(v*dt));
    N2 = round(len2/(v*dt));
    
    VCZ_interp_rob1 = [VCZ_interp_rob1; [linspace(VCZ_rob1(i,1), VCZ_rob1(i+1,1), N1)',...
                                             linspace(VCZ_rob1(i,2), VCZ_rob1(i+1,2), N1)'] ];
    VCZ_interp_rob2 = [VCZ_interp_rob2; [linspace(VCZ_rob2(i,1), VCZ_rob2(i+1,1), N2)',...
                                             linspace(VCZ_rob2(i,2), VCZ_rob2(i+1,2), N2)'] ];
end

Lmax = max(size(VCZ_interp_rob1,1), size(VCZ_interp_rob2,1));
VCZ_interp_rob1(end+1:Lmax,:) = repmat(VCZ_interp_rob1(end,:), Lmax-size(VCZ_interp_rob1,1), 1);
VCZ_interp_rob2(end+1:Lmax,:) = repmat(VCZ_interp_rob2(end,:), Lmax-size(VCZ_interp_rob2,1), 1);

%% Solve for robot control
lambda1 = 0.8;
lambda2 = 0.8;

robot1 = zeros(size(VCZ_interp_rob1, 1),2);
robot2 = zeros(size(VCZ_interp_rob1, 1),2);
drobot1 = zeros(size(VCZ_interp_rob1, 1),2);
drobot2 = zeros(size(VCZ_interp_rob1, 1),2);
robot_control1 = zeros(size(VCZ_interp_rob1, 1),2);
robot_control2 = zeros(size(VCZ_interp_rob1, 1),2);

robot1(1,:) = [9.25, 9.25];
robot2(1,:) = [5.25, 1.25];

for i=2:size(VCZ_interp_rob1, 1)

    robot_control1(i,:) = control(VCZ_interp_rob1(i-1,:), lambda1, robot1(i-1,:), drobot1(i-1,:));
    robot_control2(i,:) = control(VCZ_interp_rob2(i-1,:), lambda2, robot2(i-1,:), drobot2(i-1,:));
    
    drobot1(i,:) = drobot1(i-1,:) + dt*robot_control1(i,:);
    drobot2(i,:) = drobot2(i-1,:) + dt*robot_control2(i,:);

    robot1(i,:) = robot1(i-1,:) + dt*drobot1(i,:);
    robot2(i,:) = robot2(i-1,:) + dt*drobot2(i,:);
end

%% Animate
figure(1)
clf;
% Setup the main trajectory plot
% subplot(2,4,[1,5])
hold on;
grid on; grid minor; box on; axis equal;
axis([2.15 10.15 0 11.25]);
xlabel('$x (m)$', 'FontSize', 18, 'Interpreter', 'latex');
ylabel('$y (m)$', 'FontSize', 18, 'Interpreter', 'latex');

% 1. Draw Static Elements (Targets and Obstacles) once
for count = 1:2
    rectangle('Position',[f_targets(count,1), f_targets(count,2), ...
              f_targets(count,3)-f_targets(count,1), f_targets(count,4)-f_targets(count,2)], ...
              'FaceColor', 'g', 'EdgeColor', 'g', 'FaceAlpha', 0.5, 'HandleVisibility','off');
    dOx = 0.2; dOy = 1;
    if count == 1
        rectangle('Position',[f_obstacles(count,1), f_obstacles(count,2)+dOy, ...
                  f_obstacles(count,3)-f_obstacles(count,1), f_obstacles(count,4)-f_obstacles(count,2)-2*dOy], ...
                  'FaceColor', 'r', 'EdgeColor', 'r', 'FaceAlpha', 0.5, 'HandleVisibility','off');
    else
        rectangle('Position',[f_obstacles(count,1)+dOx, f_obstacles(count,2)+dOy, ...
                  f_obstacles(count,3)-f_obstacles(count,1)-dOx, f_obstacles(count,4)-f_obstacles(count,2)-2*dOy], ...
                  'FaceColor', 'r', 'EdgeColor', 'r', 'FaceAlpha', 0.5, 'HandleVisibility','off');
    end
end

% 2. Initialize Graphic Objects (Handles)
hRob1 = plot(robot1(1,1), robot1(1,2), 'o', 'MarkerFaceColor', '#0916C8', 'MarkerEdgeColor','none', 'HandleVisibility','off');
hRob2 = plot(robot2(1,1), robot2(1,2), 'o', 'MarkerFaceColor', '#F59127', 'MarkerEdgeColor','none', 'HandleVisibility','off');
hPath1 = plot(robot1(1,1), robot1(1,2), 'Color', '#0916C8', 'LineWidth', 2, 'DisplayName','Robot-1');
hPath2 = plot(robot2(1,1), robot2(1,2), 'Color', '#F59127', 'LineWidth', 2, 'DisplayName','Robot-2');

for i = 1:50:size(VCZ_interp_rob1, 1)
    % Update Robot Positions
    set(hRob1, 'XData', robot1(i,1), 'YData', robot1(i,2));
    set(hRob2, 'XData', robot2(i,1), 'YData', robot2(i,2));
    
    % Update Trajectory Lines
    set(hPath1, 'XData', robot1(1:i, 1), 'YData', robot1(1:i, 2));
    set(hPath2, 'XData', robot2(1:i, 1), 'YData', robot2(1:i, 2));
    
    drawnow; 
end
legend('FontName', 'Latin Modern Roman','Interpreter', 'latex','Location','best')
%% Other Plots
figure(2)
t = 0:dt:dt*(Lmax-1);

subplot(2,3,1)
hold on;
plot(t, robot1(:,1), 'DisplayName', '$x_{1,1}(t)$', 'Color', 'b', 'LineWidth', 3); 
plot(t, robot2(:,1), 'DisplayName', '$x_{2,1}(t)$', 'Color', '#FFA500', 'LineWidth', 3); 
plot(t, VCZ_interp_rob1(:,1), 'DisplayName', '$\xi_1(t)$', 'Color', 'k', 'LineStyle', ':', 'LineWidth', 3); 
plot(t, VCZ_interp_rob2(:,1), 'HandleVisibility', 'off', 'Color', 'k', 'LineStyle', ':', 'LineWidth', 3); 
set(gca,'FontSize', 18, 'FontName', 'Latin Modern Roman', 'TickLabelInterpreter', 'latex')
xlabel('$$t(s)$$', 'FontSize', 18,Interpreter ='latex');
ylabel('Dimension 1', 'FontSize', 18,Interpreter ='latex');
legend('FontName', 'Latin Modern Roman','Interpreter', 'latex','Location','best')
grid on; box on;
yticklabels({'0.6','1.3','2.0','2.7','3.4'})

subplot(2,3,4)
hold on;
plot(t, robot1(:,2), 'DisplayName', '$x_{1,2}(t)$', 'Color', 'b', 'LineWidth', 3); 
plot(t, robot2(:,2), 'DisplayName', '$x_{2,2}(t)$', 'Color', '#FFA500', 'LineWidth', 3); 
plot(t, VCZ_interp_rob1(:,2), 'DisplayName', '$\xi_2(t)$', 'Color', 'k', 'LineStyle', ':', 'LineWidth', 3); 
plot(t, VCZ_interp_rob2(:,2), 'HandleVisibility', 'off', 'Color', 'k', 'LineStyle', ':', 'LineWidth', 3); 
set(gca,'FontSize', 18, 'FontName', 'Latin Modern Roman', 'TickLabelInterpreter', 'latex')
xlabel('$$t(s)$$', 'FontSize', 18,Interpreter ='latex');
ylabel('Dimension 2', 'FontSize', 18,Interpreter ='latex');
legend('FontName', 'Latin Modern Roman','Interpreter', 'latex','Location','best')
grid on; box on;
yticklabels({'0.0', '0.8','1.6','2.4','3.2','4.0'})

subplot(2,3,2)
hold on;
plot(t, drobot1(:,1), 'DisplayName', '$\dot{x}_{1,1}(t)$', 'Color', 'b', 'LineWidth', 2); 
plot(t, drobot2(:,1), 'DisplayName', '$\dot{x}_{2,1}(t)$', 'Color', '#FFA500', 'LineWidth', 2); 
set(gca,'FontSize', 18, 'FontName', 'Latin Modern Roman', 'TickLabelInterpreter', 'latex')
xlabel('$$t(s)$$', 'FontSize', 18,Interpreter ='latex');
ylabel('Dimension 1', 'FontSize', 18,Interpreter ='latex');
legend('FontName', 'Latin Modern Roman','Interpreter', 'latex','Location','best')
grid on; box on;
yticklabels({'-0.06','-0.03','0.0','0.03','0.06'})

subplot(2,3,5)
hold on;
plot(t, drobot1(:,2), 'DisplayName', '$\dot{x}_{1,2}(t)$', 'Color', 'b', 'LineWidth', 2); 
plot(t, drobot2(:,2), 'DisplayName', '$\dot{x}_{2,2}(t)$', 'Color', '#FFA500', 'LineWidth', 2); 
set(gca,'FontSize', 18, 'FontName', 'Latin Modern Roman', 'TickLabelInterpreter', 'latex')
xlabel('$$t(s)$$', 'FontSize', 18,Interpreter ='latex');
ylabel('Dimension 2', 'FontSize', 18,Interpreter ='latex');
legend('FontName', 'Latin Modern Roman','Interpreter', 'latex','Location','best')
grid on; box on;
yticklabels({'-0.08','-0.04','0.0','0.04','0.08'})

subplot(2,3,3)
hold on;
plot(t, robot_control1(:,1), 'DisplayName', '$\tau_{1,1}(t)$', 'Color', 'b', 'LineWidth', 2); 
plot(t, robot_control2(:,1), 'DisplayName', '$\tau_{2,1}(t)$', 'Color', '#FFA500', 'LineWidth', 2); 
set(gca,'FontSize', 18, 'FontName', 'Latin Modern Roman', 'TickLabelInterpreter', 'latex')
xlabel('$$t(s)$$', 'FontSize', 18,Interpreter ='latex');
ylabel('Dimension 1', 'FontSize', 18,Interpreter ='latex');
legend('FontName', 'Latin Modern Roman','Interpreter', 'latex','Location','best')
grid on; box on;
yticklabels({'-0.06','-0.03','0.0','0.03','0.06'})

subplot(2,3,6)
hold on;
plot(t, robot_control1(:,2), 'DisplayName', '$\tau_{1,2}(t)$', 'Color', 'b', 'LineWidth', 2); 
plot(t, robot_control2(:,2), 'DisplayName', '$\tau_{2,2}(t)$', 'Color', '#FFA500', 'LineWidth', 2); 
set(gca,'FontSize', 18, 'FontName', 'Latin Modern Roman', 'TickLabelInterpreter', 'latex')
xlabel('$$t(s)$$', 'FontSize', 18,Interpreter ='latex');
ylabel('Dimension 2', 'FontSize', 18,Interpreter ='latex');
legend('FontName', 'Latin Modern Roman','Interpreter', 'latex','Location','best')
grid on; box on;
yticklabels({'-0.04','-0.02','0.0','0.02','0.04'})


%%
function acc = control(VCZ, lambda, robot, drobot)
% u = 10*(tube-robot);

a = 5;

ub = 1;
ab = 1;
% u = -ub*tanh(a*e).*(1-exp(-a*a*e.^2)) * (robot - tube) / norm(robot - tube);

eu = norm(robot - VCZ)/lambda;
% ur = -ub * (robot - VCZ) * log((1+eu)/(1-eu));
ur = -ub * (robot - VCZ) * tanh(a*eu)^3;
ur = real(ur);

ea = norm(drobot - ur)/0.02;
% acc = -ab * (drobot - ur) * log((1+ea)/(1-ea));
acc = -ab * (drobot - ur) * tanh(a*eu)^3;
acc = real(acc);

end