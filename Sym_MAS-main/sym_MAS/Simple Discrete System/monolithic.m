clear all;
close all;

%%%% FOR THE DYNAMICS x(k+1)=x(k)+u_x(k), y(k+1)=y(k)+u_y(k) %%%%

% Constraints on state space
bound_x_1 = [1 5];
bound_y_1 = [1 5];
bound_x_2 = [1 5];
bound_y_2 = [1 5];
bound = [bound_x_1; bound_y_1; bound_x_2; bound_y_2];

%Constraints on the input space
bound_u = [-2 0; -1 0; 1 0; 2 0; 0 -2; 0 -1; 0 1; 0 2];
n_u = numel(bound_u)/2;

% Space discretization
n_x = 5;
n_y = 5;
L = 1;
d = 1;
% Abstraction
disp('')
disp('Step 1: Computation of the abstraction''s transition relation')
disp(' ')

tic

Delta=logical(sparse(n_x*n_y*n_x*n_y*n_u*n_u,n_x*n_y*n_x*n_y))

% Compute the automata
pb = waitbar(0);
for i1=1:n_x
    for i2=1:n_y
        for i3=1:n_x
            for i4=1:n_y
                for h1=1:n_u
                    for h2=1:n_u
                        succ_x_1 = i1+bound_u(h1,1);
                        succ_y_1 = i2+bound_u(h1,2);
                        succ_x_2 = i3+bound_u(h2,1);
                        succ_y_2 = i4+bound_u(h2,2);
                        if((succ_x_1 >= bound(1,1))&&(succ_x_1 <= bound(1,2)) && (succ_y_1 >= bound(2,1))&&(succ_y_1 <= bound(2,2)) && (succ_x_2 >= bound(3,1))&&(succ_x_2 <= bound(3,2)) && (succ_y_2 >= bound(4,1))&&(succ_y_2 <= bound(4,2)))
                            Delta((i1-1)*n_y*n_x*n_y*n_u*n_u+(i2-1)*n_x*n_y*n_u*n_u+(i3-1)*n_y*n_u*n_u+(i4-1)*n_u*n_u+(h1-1)*n_u+h2, (succ_x_1-1)*n_y*n_x*n_y+(succ_y_1-1)*n_x*n_y+(succ_x_2-1)*n_y+succ_y_2) = 1;
                        end
                    end
                end
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
Cont=logical(zeros((n_x)*n_y*n_x*n_y,(n_u)*n_u)); % create the matrix structure (defined only with 0,1) 
for i=1:(n_x)*n_y*n_x*n_y % checking admissible modes for any state
    for h=1:(n_u)*n_u % for any control mode
        succ=find(Delta((i-1)*n_u*n_u+h,:));
        if ~isempty(succ)
          Cont(i,h)=1;
        end
    end
end
ContA=Cont;

%%%% DEFINITION OF OBSTACLE SET %%%%
obstacle_sets = [];
for i=1:n_x
    for j=1:n_y
        for k=1:n_x
            for l=1:n_y
                if(sqrt((i-k)^2+(j-l)^2)-L-d<0)
                    obstacle_sets(end+1) = (i-1)*n_y*n_x*n_y+(j-1)*n_x*n_y+(k-1)*n_y+l;
                end
            end
        end
    end
end

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

%%%%% FOR REACHABILITY %%%%%
Contp = logical(zeros((n_x)*n_y*n_x*n_y,(n_u)*n_u));
%%% Reachability Specification. Consider a region
reach_states=[(5-1)*n_y*n_x*n_y+(5-1)*n_x*n_y+(1-1)*n_y+1];
reach_state_target=reach_states;
while ~all(Cont(:)==Contp(:))
    iter=iter+1;
    txt='    Iteration %u\n'; 
    fprintf(txt,iter)
    Contp=Cont;
    
    mode_idxs = find(ContA);
    for mode_idx=1:size(mode_idxs, 1)
        [mode_r, mode_c] = ind2sub(size(ContA), mode_idxs(mode_idx));
        succ = find(Delta((mode_r-1)*n_u*n_u+mode_c, :));
        if ~ismember(mode_r, obstacle_sets)
            if ~ismember(succ, reach_states)
                Cont(mode_r, mode_c) = 0;
            else
                Cont(mode_r, mode_c) = 1;
                if ~ismember(mode_r, reach_states)
                    reach_states(end+1) = mode_r;
                end
            end
        else
            Cont(mode_r, mode_c) = 0;
        end
    end
    
    
%     for i=1:(n_x)*n_y*n_x*n_y
%         mode=find(ContA(i,:));
%         if ~isempty(mode)
%             for k=mode
%                 succ=find(Delta((i-1)*n_u*n_u+k,:));
%                 if ~ismember(i,obstacle_sets)
%                     if ~isempty(succ)
%                         for ip=succ
%                             if (~(ismember(ip,reach_states)))
%                                 Cont(i,k)=0;
%                             else
%                                 Cont(i,k)=1;
%                                 if ~any(i==reach_states)
%                                     reach_states(end+1) = i;
%                                 end
%                             end
%                         end
%                     end
%                 else
%                     Cont(i,k)=0;
%                 end
%             end
%         end
%     end
end

toc
disp(' ')
save('Monolithic_Controller','Cont')
winning_domain=zeros(nnz(Cont),6);
iter = 1;
for i = 1:n_x
    for j = 1:n_y
        for l = 1:n_x
            for m = 1:n_y
                for k = 1:n_u
                    for o = 1:n_u
                        if(Cont((i-1)*n_y*n_x*n_y+(j-1)*n_x*n_y+(l-1)*n_y+m,(k-1)*n_u+o))
                            winning_domain(iter,:) = [i j l m k o];
                            iter=iter+1;
                        end
                    end
                end
            end
        end
    end
end

pos_x_1=2;
pos_y_1=2;
pos_x_2=4;
pos_y_2=5;
U_x = find(Cont((pos_x_1-1)*n_y*n_x*n_y+(pos_y_1-1)*n_x*n_y+(pos_x_2-1)*n_y+pos_y_2,:));
pos_id=(pos_x_1-1)*n_y*n_x*n_y+(pos_y_1-1)*n_x*n_y+(pos_x_2-1)*n_y+pos_y_2;

% Simulation
while (reach_state_target~=pos_id)
    pos_id=find(Delta((pos_id-1)*n_u*n_u+U_x(1),:));
    pos = pos_id;
    if(rem(pos,n_y)==0)
        pos_y_now_2 = 5;
        pos_y_2(end+1) = 5;
    else
        pos_y_now_2 = rem(pos,n_y);
        pos_y_2(end+1) = rem(pos,n_y);
    end
    pos = (pos-pos_y_now_2)/n_y+1;
    if(rem(pos,n_x)==0)
        pos_x_now_2 = 5;
        pos_x_2(end+1) = 5;
    else
        pos_x_now_2 = rem(pos,n_x);
        pos_x_2(end+1) = rem(pos,n_x);
    end
    pos = (pos-pos_x_now_2)/n_x+1;
    if(rem(pos,n_y)==0)
        pos_y_now_1 = 5;
        pos_y_1(end+1) = 5;
    else
        pos_y_now_1 = rem(pos,n_x);
        pos_y_1(end+1) = rem(pos,n_x);
    end
    pos_x_now_1 = (pos-pos_y_now_1)/n_y+1;
    pos_x_1(end+1) = (pos-pos_y_now_1)/n_y+1;
    U_x = find(Cont(pos_id,:));
end

plot(pos_x_1,pos_y_1)
hold on
plot(pos_x_2,pos_y_2)