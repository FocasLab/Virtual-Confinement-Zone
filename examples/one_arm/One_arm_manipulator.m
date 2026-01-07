%
% VCZ.m
% you need to 1. have the mexfiles compiled 
%             2. run the ./dcdc binary first 
%
% so that the file: controller.scs is created
%
addpath(genpath('../../mfiles'))

clc;
clear;
clf;

%% simulation

% initial state
th0=-0.15;
omega0 = 0;

x0 = [th0; omega0];

% load controller from file
controller=StaticController('controller');

% simulate closed loop system
th_e=th0;
om_e=omega0;

dt = 1e-4;
tau_s=dt;
t_final = 10;
T=(0:dt:t_final)'; 

%% VCZ

tspan = [0, t_final];  
% h = dt;        
% controller=readmatrix("controller.csv");
% [t, th_e, om_e] = rk4_solver_env(@envelope_ode, tspan, th0, controller, h);  % VCZ dynamics

for i=2:length(T)  
    u=controller.control(th_e(end,:));

    %-------------here choose your controller input-------------%
    %in=u(end,:);
    in= max(u);
    %-----------------------------------------------------------%

    [t x]=ode45(@envelope_ode,[0 tau_s], th_e(end,:), odeset('abstol',1e-10,'reltol',1e-10),in');

    th_e=[th_e; x(end,:)];
    om_e=[om_e; in];
end

%%
th = zeros(length(th_e),1);
om = zeros(length(th_e),1);
om_d = zeros(length(th_e),1);
tau = zeros(length(th_e),1);
th(1) = th0;

lam = 0.018;
taub = 5;
omb = 0.16;

for i=2:length(T)
    q = [th(i-1); om(i-1)];
    dq = pend_ode(q,tau(i-1));
    th(i) = th(i-1) + dt*dq(1);
    om(i) = om(i-1) + dt*dq(2);
    % th(i) = th(i-1) + dt*om_d(i-1);
    % om(i) = om(i-1) + dt*tau(i-1);

    et = abs(th(i)-th_e(i))/lam;
    om_d(i) = -omb*psi(et)*(th(i)-th_e(i))/abs(th(i)-th_e(i));
    
    eo = abs(om(i)-om_d(i))/lam;
    tau(i) = -taub*psi(eo)*(om(i)-om_d(i))/abs(om(i)-om_d(i));
end

%% SCOTS 
figure(1);
clf;
tiledlayout(1,3,'TileSpacing','compact','Padding','compact');
nexttile;
hold on;
grid on;
box on
plot(T, th_e,'k:', 'LineWidth', 3);
plot(T, th,'b', 'LineWidth', 3);
set(gca,'FontSize',25, 'FontName', 'Latin Modern Roman', 'TickLabelInterpreter', 'latex')
xlabel('$$t(s)$$', 'FontSize', 25,Interpreter ='latex');
ylabel('$$\theta(rad)$$', 'FontSize', 25,Interpreter ='latex');
% rectangle('Position',[0,-0.2,t_final,0.4],'FaceColor',[0.04 1 1], 'FaceAlpha',0.1,'LineWidth',1.5,'HandleVisibility','off')
legend('$\xi(t)$ - VCZ Center','$\theta(t)$ - Pendulum Angle', 'FontName', 'Latin Modern Roman','Interpreter', 'latex','Location','best')

nexttile;
hold on;
grid on;
box on
% plot(T, om_e, 'k:', 'LineWidth', 3);
% plot(T, om_d, 'k', 'LineWidth', 3);
plot(T, om, 'b', 'LineWidth', 3);
set(gca,'FontSize',25, 'FontName', 'Latin Modern Roman', 'TickLabelInterpreter', 'latex')
xlabel('$$t(s)$$', 'FontSize', 25,Interpreter ='latex');
ylabel('$$\omega(rad/s)$$', 'FontSize', 25,Interpreter ='latex');
% rectangle('Position',[0,-0.5,t_final,1],'FaceColor',[0.04 1 1], 'FaceAlpha',0.1,'LineWidth',1.5,'HandleVisibility','off')
legend('$\omega(t)$ - Proposed VCZ method', 'FontName', 'Latin Modern Roman','Interpreter', 'latex','Location','best')

nexttile;
hold on
grid on
box on
plot(T, tau,'b', 'LineWidth', 3);
set(gca,'FontSize',25, 'FontName', 'Latin Modern Roman', 'TickLabelInterpreter', 'latex')
xlabel('$$t(s)$$', 'FontSize', 25,Interpreter ='latex');
ylabel('$$u(Nm)$$', 'FontSize', 25,Interpreter ='latex');
legend('$\tau(t)$ - Proposed VCZ method', 'FontName', 'Latin Modern Roman','Interpreter', 'latex','Location','best')
%%

function p = psi(e)
    a = 5;
    % p = tanh(a*e);
    % p = tanh(a*e)*(1-exp(-(a*e)^2));
    p = tanh(a*e)^3;
end

%%
function dxdt = envelope_ode(t, x, u)
    % parameter initialization
    dxdt = u;
end


%%
function dxdt = pend_ode(x,u)
    dxdt = zeros(2,1);
    g=9.81;
    l=1;
    m=1;
    dxdt(1)=x(2);
    dxdt(2)=(-1.5*g/l)*(x(1))+3*u/(m*l*l);
end

function [T, X,U_mat] = rk4_solver(odefun, tspan, x0, data, h)
    t0 = tspan(1);
    tf = tspan(2);
    T = t0:h:tf;
    U_mat = [0];
    X = zeros(length(T), length(x0));
    theta_quantization = 0.01;
    velocity_quantization = 0.01;
    states = data(:, 1:2);
    input_torque = data(:, 3:end);
    X(1, :) = x0;
    scots_input=0;
    for i = 1:(length(T)-1)
        joint_info = X(i, :); 
        stat1 = find(abs(joint_info(1) - states(:, 1)) <= theta_quantization/2);
        stat2 = find(abs(joint_info(2) - states(:, 2)) <= velocity_quantization/2);
        com_state = intersect(stat1, stat2);
        if ~isempty(com_state)
            % [~, min_index] = max(abs(input_torque(com_state(1), :)), [], 'omitnan');
            % scots_input = input_torque(com_state, min_index);

            scots_input = max(input_torque(com_state(1), :), [], 'omitnan');
        else
            fprintf('State not found: Theta = %.4f, Velocity = %.4f\n', joint_info(1), joint_info(2));
            % scots_input = scots_input;  
        end
        x = X(i, :)';
        k1 = odefun(x, scots_input);
        k2 = odefun( x + h/2 * k1, scots_input);
        k3 = odefun( x + h/2 * k2, scots_input);
        k4 = odefun( x + h * k3, scots_input);
        x_next = x + (h/6) * (k1 + 2*k2 + 2*k3 + k4);
        X(i+1, :) = x_next';
        U_mat(end+1)=scots_input;
    end
end
