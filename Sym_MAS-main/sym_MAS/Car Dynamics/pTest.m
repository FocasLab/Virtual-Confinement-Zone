% General parameters
clear all;
close all;

tau=2.1;

pool = parpool('Threads')

%bound
bound_P=[-1000 0];

% Constraints on State/Bounds on the Robot
bound_x = [1 5];
bound_y = [1 5];
bound_theta = [-pi pi];
bound = [bound_x;bound_y;bound_theta];

% Input Constraints of the Robot
bound_v = [-1 1];
bound_w = [-0.5 0.5];
bound_u = [bound_v;bound_w];

% Space discretization
n_x=10;% numbers of intervals
n_y=10;
n_theta=10;

d_x=(bound_x(:,2)-bound_x(:,1))./(n_x); % size of the interval
d_y=(bound_y(:,2)-bound_y(:,1))./(n_y);
d_theta=(bound_theta(:,2)-bound_theta(:,1))./(n_theta);
d_states = [d_x;d_y;d_theta];

% Input discretization
n_v = 5;
n_w = 5;
d_v=(bound_v(:,2)-bound_v(:,1))./(n_v);
d_w=(bound_w(:,2)-bound_w(:,1))./(n_w);

u_values1=bound_u(1,1):(bound_u(1,2)-bound_u(1,1))/(n_v):bound_u(1,2);
u_values2=bound_u(2,1):(bound_u(2,2)-bound_u(2,1))/(n_w):bound_u(2,2);

% Abstraction
disp(' ')
disp('Step 1: Computation of the abstraction''s transition relation')
disp(' ')

tic
% Creating a Sparse matrix denoting transition
Delta=logical(sparse((n_x*n_y*n_theta)*(n_v)*(n_w),(n_x*n_y*n_theta))); % create the matrix structure (defined only with 0,1)
for i1=1:n_x
    for i2=1:n_y
        for i3 = 1:n_theta
            for h1 = 1:n_v
                for h2 = 1:n_w
                    parfeval(pool, @pMain1, 0, i1,i2,i3,u_values1(h1),u_values2(h2),n_x,n_y,n_theta,n_v,n_w)
                end
            end
        end
    end
end