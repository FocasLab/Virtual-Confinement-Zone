clear all;
close all;

%%%% FOR THE DYNAMICS x(k+1)=x(k)+u_x(k), x(k+1)=x(k)+u_y(k) %%%%

% Data change
n_lim = 5;

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

Delta=logical(sparse(n_x*n_y*n_u,n_x*n_y));

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

% Controller Synthesis
disp('Step 2: Controller synthesis')
disp(' ')
tic
disp('  Fixed point algorithm:')

% Controller Synthesis
Cont=logical(zeros((n_x)*n_y,(n_u))); % create the matrix structure (defined only with 0,1) 
for i=1:(n_x)*n_y % checking admissible modes for any state
    for h=1:(n_u) % for any control mode
        succ=find(Delta((i-1)*n_u+h,:));
        if ~isempty(succ)
          Cont(i,h)=1;
        end
    end
end
ContA=Cont;

iter=0;
%%%% FOR SAFETY SPECIFICATIONS %%%%
% Contp=logical(zeros(n_x*n_y,n_u)); % eliminate transitions reaching blocking states
% while ~all(Cont(:)==Contp(:))
%     iter=iter+1;
%     txt='    Iteration %u\n'; 
%     fprintf(txt,iter)
%     Contp=Cont;

%     % Compute all the successors from controller matrix
%     for i=1:n_x*n_y
%         mode=find(Contp(i,:));
%         if ~isempty(mode)
%             for k=mode
%                 succ=find(Delta((i-1)*n_u+k,:));
%                 if ~isempty(succ)
%                     for ip=succ
%                         if ~any(Contp(ip,:))
%                             Cont(i,k)=0;
%                             break;
%                         end
%                     end
%                 end
%             end
%         end
%     end
% end

t_r1 = [5, 5];

%%%%% FOR REACHABILITY %%%%%
Contp = logical(zeros((n_x)*n_y,(n_u)));
%%% Reachability Specification. Consider a region
reach_states=(t_r1(1)-1)*n_y+t_r1(2);
reach_state_target=reach_states;
while ~all(Cont(:)==Contp(:))
    iter=iter+1;
    txt='               Iteration %u\n'; 
    fprintf(txt,iter)
    Contp=Cont;
    for i=1:(n_x)*n_y
        mode=find(ContA(i,:));
        if ~isempty(mode)
            for k=mode
                succ=find(Delta((i-1)*n_u+k,:));
                if ~isempty(succ)
                    for ip=succ
                        if ~(ismember(ip,reach_states))
                            Cont(i,k)=0;
                        else
                            Cont(i,k)=1;
                            if ~any(i==reach_states)
                                reach_states(end+1) = i;
                            end
                        end
                    end
                end
            end
        end
    end
end

% Converting the Controller into Transition System
controlled_Delta1=logical(sparse(n_x*n_y*n_u,n_x*n_y));
for i=1:(n_x*n_y)
    for h=1:(n_u)
        if(Cont(i,h))
            controlled_Delta1((i-1)*n_u+h,:) = Delta((i-1)*n_u+h,:);
        end
    end
end
toc

disp(' ')
winning_domain=zeros(nnz(Cont),3);
iter = 1;
for i = 1:n_x
    for j = 1:n_y
        for k = 1:n_u
            if(Cont(((i-1)*n_y+j),k))
                winning_domain(iter,:) = [i j k];
                iter=iter+1;
            end
        end
    end
end

save('Controlled_System1','controlled_Delta1');
save('Target_1', 't_r1');

% disp(winning_domain)
init_pos = [3 3];
pos_x=[init_pos(1)];
pos_y=[init_pos(2)];
U_x = find(Cont((init_pos(1)-1)*n_y+init_pos(2),:));
pos=(init_pos(1)-1)*n_y+init_pos(2);

% Simulation
while (reach_state_target~=pos)
%     idx = randi(size(U_x, 2));
    pos=find(Delta((pos-1)*n_u+U_x(1),:));
    if(rem(pos,n_y)==0)
        pos_y_now = n_y;
        pos_y(end+1) = n_y;
    else
        pos_y_now = rem(pos,n_y);
        pos_y(end+1) = rem(pos,n_y);
    end
    pos_x(end+1) = (pos-pos_y_now)/n_y+1;
    U_x = find(Cont(pos,:));
end

plot(pos_x,pos_y)
xlim([1 n_lim])
ylim([1 n_lim])
grid on
grid minor
