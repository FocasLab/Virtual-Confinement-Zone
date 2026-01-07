clc;
clear;
clf;

addpath(genpath('../../mfiles'))

VCZ = StaticController('controller'); % 2D: [theta1, theta2]
dom2 = VCZ.domain;

figure(1); clf; hold on;
scatter(dom2(:,1), dom2(:,2), 12, 'b.', 'DisplayName', 'VCZ winning set');
xlabel('\theta_1 (rad)'); ylabel('\theta_2 (rad)');
legend('Location','northoutside'); grid on; box on; axis equal;
