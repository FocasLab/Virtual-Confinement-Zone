%%% Test 1
clear all;
close all;

%%%% FOR THE DYNAMICS x(k+1)=x(k)+u_x(k), x(k+1)=x(k)+u_y(k) %%%%

% Data change
n_lim = 25;

% Constraints on state space
bound_x = [1 n_lim];
bound_y = [1 n_lim];
bound = [bound_x; bound_y];

%Constraints on the input space
bound_u = [-2 0; -1 0; 1 0; 2 0; 0 -2; 0 -1; 0 1; 0 2];
n_u = numel(bound_u)/2;

% Space discretization
n_x = n_lim;
n_y = n_lim;

d_x=(bound_x(:,2)-bound_x(:,1))./(n_x); % size of the interval
d_y=(bound_y(:,2)-bound_y(:,1))./(n_y);
d_states = [d_x;d_y];

% Abstraction
disp('')
disp('Step 1: Computation of the abstraction''s transition relation')
disp(' ')

tic

Delta=gpuArray(sparse(n_x*n_y*n_u,n_x*n_y));

% Compute the automata
pb = waitbar(0);
for i1=1:n_x
    for i2=1:n_y
        for h1=1:n_u
            succ_x = i1+bound_u(h1,1);
            succ_y = i2+bound_u(h1,2);
            if((succ_x >= bound(1,1))&&(succ_x <= bound(1,2)) && (succ_y >= bound(2,1))&&(succ_y <= bound(2,2)))
                Delta((i1-1)*n_y*n_u+(i2-1)*n_u+h1, (succ_x-1)*n_y+succ_y) = 1;
        
            end    
        end
    end
end

close(pb);

whos Delta
toc
disp(' ');