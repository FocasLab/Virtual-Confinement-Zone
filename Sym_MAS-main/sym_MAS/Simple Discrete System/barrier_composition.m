clear all;
close all;

%%%% FOR THE DYNAMICS x(k+1)=x(k)+u_x(k), x(k+1)=x(k)+u_y(k) %%%%

% Constraints on state space
bound_x = [1 5];
bound_y = [1 5];
bound = [bound_x; bound_y];

%Constraints on the input space
bound_u = [-2 0; -1 0; 1 0; 2 0; 0 -2; 0 -1; 0 1; 0 2];
n_u = numel(bound_u)/2;

% Space discretization
n_x = 5;
n_y = 5;

% size of the interval
d_x=(bound_x(:,2)-bound_x(:,1))./(n_x);
d_y=(bound_y(:,2)-bound_y(:,1))./(n_y);
d_states = [d_x;d_y];

%%%%% Barrier Certificate = d-norm(x_i-x_j)-L %%%%%
d=1; % Distance to be maintained
L=1; % Parameter for abstraction of barrier

load('Controlled_System1');
load('Controlled_System2');
%load('Delta3');
%load('Delta4');
%load('Delta5');

tic
%%% Composition of Controlled system
iter = 0;
Delta=logical(sparse(n_x*n_y*n_x*n_y*n_u*n_u, n_x*n_y*n_x*n_y));
p2 = waitbar(0,'Abstraction in progress...');
% No of For loops depends on the number of sub systems
for i1=1:n_x
    waitbar(i1/n_x,p2,sprintf('Computing the abstraction of the source subsystem - %2.2f',i1/n_x))
    for i2=1:n_y       
        for i3=1:n_x
            for i4=1:n_y
                for h1=1:n_u
                    for h2=1:n_u
                        for ip1=1:n_x
                            for ip2=1:n_y
                                for ip3=1:n_x
                                    for ip4=1:n_y
                                        if((sqrt((i1-i3)^2+(i2-i4)^2))-L-d)>=0
                                            row_1 = (i1-1)*n_y*n_u+(i2-1)*n_u+h1;
                                            col_1 = (ip1-1)*n_y+ip2;
                                            if(controlled_Delta1(row_1,col_1))
                                                r1=[ip1;ip2];
                                            else
                                                continue;
                                            end
                                            row_2 = (i3-1)*n_y*n_u+(i4-1)*n_u+h2;
                                            col_2 = (ip3-1)*n_y+ip4;
                                            if(controlled_Delta2(row_2,col_2))
                                                r2=[ip3;ip4];
                                            else
                                                continue;
                                            end
                                            % Euclidean distance and -1 for abs of barrier
                                                                                                    
                                            B_prime = sqrt((r1(1)-r2(1))^2+(r1(2)-r2(2))^2)-sqrt((i1-i3)^2+(i2-i4)^2);
                                            if (B_prime >= -1*(sqrt((i1-i3)^2+(i2-i4)^2)-L-d))
                                                Delta((i1-1)*n_y*n_x*n_y*n_u*n_u+(i2-1)*n_x*n_y*n_u*n_u+(i3-1)*n_y*n_u*n_u+(i4-1)*n_u*n_u+(h1-1)*n_u+h2,(ip1-1)*n_y*n_x*n_y+(ip2-1)*n_x*n_y+(ip3-1)*n_y+ip4)=1;
                                            end
                                        end
                                    end
                                end
                            end
                        end
                    end
                end
            end
        end
    end
end
toc
disp(' ')

tic
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
% load('controllersyn_py.mat')
% Cont = controller_syn;
% ContA = controller_syn;

iter=0;
%%%% FOR SAFETY SPECIFICATIONS %%%%
% Contp=logical(zeros(n_x*n_y*n_x*n_y,n_u*n_u)); % eliminate transitions reaching blocking states
% while ~all(Cont(:)==Contp(:))
%     iter=iter+1;
%     txt='    Iteration %u\n';
%     fprintf(txt,iter)
%     Contp=Cont;
% 
%     % Compute all the successors from controller matrix
%     for i=1:n_x*n_y*n_x*n_y
%         mode=find(Contp(i,:));
%         if ~isempty(mode)
%             for k=mode
%                 succ=find(Delta((i-1)*n_u*n_u+k,:));
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

% %%%%% FOR REACHABILITY %%%%%
Contp = logical(zeros((n_x)*n_y*n_x*n_y,(n_u)*n_u));
%%% Reachability Specification. Consider a region
reach_states=[(5-1)*n_y*n_x*n_y+(5-1)*n_x*n_y+(1-1)*n_y+1]
reach_state_target=reach_states;
while ~all(Cont(:)==Contp(:))
    iter=iter+1;
    txt='    Iteration %u\n'; 
    fprintf(txt,iter)
    Contp=Cont;
    for i=1:(n_x)*n_y*n_x*n_y
        mode=find(ContA(i,:));
        if ~isempty(mode)
            for k=mode
                succ=find(Delta((i-1)*n_u*n_u+k,:));
                if ~isempty(succ)
                    for ip=succ
                        if ~ismember(i,reach_states)
                            if ~(ismember(ip,reach_states))
                                Cont(i,k)=0;
                                break;
                            else
                                Cont(i,k)=1; 
                                reach_states(end+1) = i;
                            end
                        end
                    end
                end
            end
        end
    end
end

close(p2)
% load('Reach_py')
% Cont=Reach_py
toc
disp(' ')
save('Augmented_Controller','Cont')
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



% Simulation
pos_x_1=1;
pos_y_1=1;
pos_x_2=5;
pos_y_2=5;
pos_id=(pos_x_1-1)*n_y*n_x*n_y+(pos_y_1-1)*n_x*n_y+(pos_x_2-1)*n_y+pos_y_2;
U_x = find(Cont((pos_x_1-1)*n_y*n_x*n_y+(pos_y_1-1)*n_x*n_y+(pos_x_2-1)*n_y+pos_y_2,:));

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