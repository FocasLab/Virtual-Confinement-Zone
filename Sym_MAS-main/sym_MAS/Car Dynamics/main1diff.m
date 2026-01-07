% General parameters
clear all;
close all;

tau=2.1;

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

% compute the automata (reachable sets)
pb=waitbar(0);
for i1=1:n_x  % for any state interval
     waitbar(i1/n_x,pb,sprintf('Computing the abstraction of the source subsystem - %2.2f',i1/n_x))
    for i2=1:n_y
        for i3=1:n_theta
            % Extreme values of Cell/Symbol 
            x=[bound(:,1)+[(i1-1);(i2-1);(i3-1)+(bound(3,1)/d_theta)].*d_states,...
                bound(:,1)+[(i1);(i2);i3+(bound(3,1)/d_theta)].*d_states];
            
            for h1=1:n_v
                for h2=1:n_w
                     % Calculate Successors of the extreme values of symbol
                     u_values = [u_values1(h1);u_values2(h2)];
                     xp1=diff_Successor(x(:,1),u_values,bound_P(:,1),tau);
                     xp2=diff_Successor(x(:,2),u_values,bound_P(:,2),tau);
                     
                     % Check if succesors are within bounds of State space
                     if all(xp1 >= bound(:,1)) && all(xp2 <= bound(:,2))
                        minsucc=floor((xp1-bound(:,1))./d_states)+[1;1;1]; % associate successors of extrema to extremal intervals
                        maxsucc=ceil((xp2-bound(:,1))./d_states);
                        for ip1=minsucc(1):maxsucc(1) % compute the whole interval contained into the extremal intervals
                            for ip2=minsucc(2):maxsucc(2)
                                for ip3=minsucc(3):maxsucc(3)
                                    Delta((i1-1)*n_y*n_theta*n_v*n_w+(i2-1)*(n_theta)*n_v*n_w+(i3-1)*n_v*n_w+(h1-1)*n_w+h2,(ip1-1)*n_y*n_theta+(ip2-1)*n_theta+ip3)=1;  %
                                end
                            end
                        end
                     end
                end
            end
        end
    end
end


     
%end
close(pb);
 
whos Delta
toc
disp(' ')

Delta1=Delta;

% Controller Synthesis
disp('Step 2: Controller synthesis')
disp(' ')
tic
disp('  Fixed point algorithm:')
iter=0;


Cont=logical(zeros((n_x)*n_y*n_theta,(n_v)*(n_w))); % create the matrix structure (defined only with 0,1) 
for i=1:(n_x)*n_y*n_theta % checking admissible modes for any state
    for h=1:(n_v)*(n_w) % for any control mode
        succ=find(Delta((i-1)*n_v*n_w+h,:));
        if ~isempty(succ)
          Cont(i,h)=1;
        end
    end
end

ContA=Cont;

%%%% FOR SAFETY SPECIFICATIONS %%%%%
% Contp=logical(zeros((n_x)^3,(n_v)*(n_w))); % eliminate transitions reaching blocking states
% while ~all(Cont(:)==Contp(:))
%     iter=iter+1;
%     txt='    Iteration %u\n'; 
%     fprintf(txt,iter)
%     Contp=Cont;
% 
%     Compute all the successors from controller matrix
%     for i=1:(n_x)*n_y*n_theta
%         mode=find(Contp(i,:));
%         if ~isempty(mode)
%             for k=mode
%                 succ=find(Delta((i-1)*n_v*n_w+k,:));
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
Contp = logical(zeros((n_x)*n_y*n_theta,(n_v)*(n_w)));
%%% Reachability Specification. Consider a region
reach_states=[4*n_y*n_theta+2*n_theta+1, 4*n_y*n_theta+2*n_theta+2, 4*n_y*n_theta+2*n_theta+3, 4*n_y*n_theta+2*n_theta+4, 4*n_y*n_theta+2*n_theta+5, 3*n_y*n_theta+2*n_theta+1, 3*n_y*n_theta+2*n_theta+2, 3*n_y*n_theta+2*n_theta+3, 3*n_y*n_theta+2*n_theta+4, 3*n_y*n_theta+2*n_theta+5, 3*n_y*n_theta+1*n_theta+1, 3*n_y*n_theta+1*n_theta+2, 3*n_y*n_theta+1*n_theta+3, 3*n_y*n_theta+1*n_theta+4, 3*n_y*n_theta+1*n_theta+5, 4*n_y*n_theta+1*n_theta+1, 4*n_y*n_theta+1*n_theta+2, 4*n_y*n_theta+1*n_theta+3, 4*n_y*n_theta+1*n_theta+4, 4*n_y*n_theta+1*n_theta+5];
while ~all(Cont(:)==Contp(:))
    iter=iter+1;
    txt='    Iteration %u\n'; 
    fprintf(txt,iter)
    Contp=Cont;
    for i=1:(n_x)*n_y*n_theta
        mode=find(ContA(i,:));
        if ~isempty(mode)
            for k=mode
                succ=find(Delta((i-1)*n_v*n_w+k,:));
                if ~isempty(succ)
                    for ip=succ
                        if ~any(ip==reach_states)
                            Cont(i,k)=0;
                            break;
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

% ContA=Cont;

% Contp=logical(zeros((n_x)^3,(n_v)*(n_w))); % eliminate transitions reaching blocking states
% while ~all(Cont(:)==Contp(:))
%     iter=iter+1;
%     txt='    Iteration %u\n'; 
%     fprintf(txt,iter)
%     Contp=Cont;
%     for i=1:(n_x)*n_y*n_theta
%         mode=find(Contp(i,:));
%         if ~isempty(mode)
%             for k=mode
%                 succ=find(Delta((i-1)*n_v*n_w+k,:));
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

% Converting the Controller into Transition System
controlled_Delta1=logical(sparse((n_x*n_y*n_theta)*(n_v)*(n_w),(n_x*n_y*n_theta)));
for i=1:(n_x*n_y*n_theta)
    for h=1:(n_v*n_w)
        if(Cont(i,h))
            controlled_Delta1((i-1)*n_v*n_w+h,:)=Delta1((i-1)*n_v*n_w+h,:);
        else
            break;
        end
    end
end

% save('controlled_Delta1','controlled_Delta1')

disp('  Fixed point reached')
disp(' ')
whos Cont
toc
disp(' ')
%figure('Name','Controller','NumberTitle','off');
%plot_cont_HDV_pre(Cont,[n_x;n_x],[n_modes;n_modes],bound,d_x)
save('Controller1','Cont')

return