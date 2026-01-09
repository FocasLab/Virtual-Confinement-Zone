clear;
clc;

addpath(genpath('../../'))
addpath(genpath('../../mfiles'))
addpath(genpath('../../mfiles/mexfiles/'))

% load controller from file
controller=StaticController('controller');

% plot controller domain
dom=controller.domain;
[dom_r,dom_c]=size(dom);
input_scots=ones(dom_r,201)*NaN;
for i=1:dom_r
    u = controller.control(dom(i,:));
    u_row = u(:)'; % Force to row vector
    input_scots(i,1:width(u_row)) = u_row;
end
domain_Table = array2table([dom,input_scots]);
% T.Properties.VariableNames(1:length(v)) ="input_torque";
writetable(domain_Table,"controller.csv")
domain_data=readmatrix("controller.csv");
