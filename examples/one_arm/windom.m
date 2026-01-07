clc;
clear;
clf;

addpath(genpath('../../mfiles'))

% load controller from file (VCZ only)
VCZ_controller = StaticController('controller');

% plot controller domain
VCZ_dom = VCZ_controller.domain;

VCZ_dom2 = linspace(-0.1,0.1,200);
% Create full grids using repmat
VCZ_X2 = repmat(VCZ_dom, 1, length(VCZ_dom2));      % Nx200 matrix
VCZ_Y2 = repmat(VCZ_dom2, length(VCZ_dom), 1);      % Nx200 matrix

% \lambda = 0.0134
VCZ_dom3 = linspace(-0.01,0.01,200);
% Create full grids using repmat
VCZ_X3 = repmat(1.07*VCZ_dom, 1, length(VCZ_dom2));      % Nx200 matrix
VCZ_Y3 = repmat(VCZ_dom3, length(VCZ_dom), 1);      % Nx200 matrix

%% Plot

figure(1)
hold on;

% Plot VCZ data
h1 = plot(VCZ_X2, VCZ_Y2, 'b*', ...
    'LineStyle', 'none', ...
    'DisplayName', 'VCZ Approach $\lambda = 0.018$, $\bar{u} = 0.1$rad/s');

% Plot VCZ data
h2 = plot(VCZ_X3, VCZ_Y3, 'g.', ...
    'LineStyle', 'none', ...
    'DisplayName', 'VCZ Approach $\lambda = 0.009$, $\bar{u} = 0.01$rad/s');

% Axes formatting
set(gca, 'FontSize', 22, ...
         'FontName', 'Latin Modern Roman', ...
         'TickLabelInterpreter', 'latex', ...
         'Layer', 'top');   % << This makes grid lines appear on top

set(gca, ...
    'Layer', 'top', ...
    'GridColor', [1 1 1], ...      % Custom RGB color (greenish)
    'GridAlpha', 0.8, ...                  % Fully opaque
    'GridLineStyle', '--', ...            % Solid lines
    'LineWidth', 1);                   % Grid and box line width


xlabel('$$\theta (rad)$$', 'FontSize', 22, 'Interpreter', 'latex');
ylabel('$$\omega (rad/s)$$', 'FontSize', 22, 'Interpreter', 'latex');

legend([h1(1), h2(1)], 'FontName', 'Latin Modern Roman', ...
       'Interpreter', 'latex', 'Location', 'northoutside');

grid on; 
box on;
