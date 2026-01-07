% General parameters
clear all

tau=1e-4;

bound_P=[-1000 0];


%bound
bound_x = [0 10];
bound_y = [0 10];
bound_theta = [-pi pi];
bound_phi = [-pi/2 pi/2];
bound = [bound_x;bound_y;bound_theta;bound_phi];


bound_v = [-5 5];
bound_w = [-5 5];
bound_u = [bound_v;bound_w];

% Space discretization
n_x=5;% numbers of intervals
n_y=5;
n_theta=5;
n_phi=5;
d_x=(bound_x(:,2)-bound_x(:,1))./n_x; % size of the interval
d_y=(bound_y(:,2)-bound_y(:,1))./n_y;
d_theta=(bound_theta(:,2)-bound_theta(:,1))./n_theta;
d_phi=(bound_phi(:,2)-bound_phi(:,1))./n_phi;
d_states = [d_x;d_y;d_theta;d_phi];

% Input discretization
n_v = 5;
n_w = 5;
d_v=(bound_v(:,2)-bound_v(:,1))./n_v;
d_w=(bound_w(:,2)-bound_w(:,1))./n_w;

u_values1=bound_u(1,1):(bound_u(1,2)-bound_u(1,1))/(n_v):bound_u(1,2);
u_values2=bound_u(2,1):(bound_u(2,2)-bound_u(2,1))/(n_w):bound_u(2,2);

% Abstraction
disp(' ')
disp('Step 1: Computation of the abstraction''s transition relation')
disp(' ')
tic
Delta=logical(sparse((n_x*n_y*n_theta*n_phi)*(n_v)*(n_w),(n_x*n_y*n_theta*n_phi))); % create the matrix structure (defined only with 0,1)
% compute the automata (reachable sets)
pb=waitbar(0);
for i1=1:n_x  % for any state interval
     waitbar(i1/n_x,pb,sprintf('Computing the abstraction of the source subsystem - %2.2f',i1/n_x))
    for i2=1:n_y
        for i3=1:n_theta
            for i4=1:n_phi
                x=[bound(:,1)+[(i1-1);(i2-1);(i3-1);(i4-1)].*d_states,...
                    bound(:,1)+[(i1);(i2);i3;i4].*d_states];
                x(4,:)=x(4,:)+[-pi/2 -pi/2];
                for h1=1:n_v
                    for h2=1:n_w
                        u_values = [u_values1(h1);u_values2(h2)];
                         xp1=acker_Successor(x(:,1),u_values,bound_P(:,1),tau);
                         xp2=acker_Successor(x(:,2),u_values,bound_P(:,2),tau);
                         if all(xp1 >= bound(:,1)) & all(xp2 <= bound(:,2))
                            minsucc=floor((xp1-bound(:,1))./d_states)+[1;1;1;1]; % associate successors of extrema to extremal intervals
                            maxsucc=ceil((xp2-bound(:,1))./d_states);
                            for ip1=minsucc(1):maxsucc(1) % compute the whole interval contained into the extremal intervals
                                for ip2=minsucc(2):maxsucc(2)
                                    for ip3=minsucc(3):maxsucc(3)
                                        for ip4=minsucc(4):maxsucc(4)
                                            Delta((i1-1)*(n_x)^3*n_v*n_w+(i2-1)*(n_x)^2*n_v*n_w+(i3-1)*n_x*n_v*n_w+(i4-1)*n_v*n_w+(h1-1)*n_v+h2,(ip1-1)*n_x^3+(ip2-1)*n_x^2+(ip3-1)*n_x+ip4)=1;
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


     
%      end

          close(pb)         ;
 
whos Delta
toc
disp(' ')

Delta2=Delta; 

% Controller Synthesis
disp('Step 2: Controller synthesis')
disp(' ')
tic
disp('  Fixed point algorithm:')
iter=0;
Cont=logical(zeros((n_x)^4,(n_v)*(n_w))); % create the matrix structure (defined only with 0,1) 
for i=1:(n_x)^4 % checking admissible modes for any state
    for h=1:(n_v)*(n_w) % for any control mode
        succ=find(Delta((i-1)*n_v*n_w+h,:));
        if ~isempty(succ)
          Cont(i,h)=1;
        end
    end
end
   
ContA=Cont;

Contp=logical(zeros((n_x)^4,(n_v)*(n_w))); % eliminate transitions reaching blocking states
while ~all(Cont(:)==Contp(:))
    iter=iter+1;
    txt='    Iteration %u\n'; 
    fprintf(txt,iter);
    Contp=Cont;
    for i=1:(n_x)^4
        mode=find(Contp(i,:));
        if ~isempty(mode)    
            for k=mode
                succ=find(Delta((i-1)*n_v*n_w+k,:));
                if ~isempty(succ)
                    for ip=succ
                        if ~any(Contp(ip,:))
                            Cont(i,k)=0;
                            break;
                        end
                    end
                end
            end
        end
    end
end

for i=1:(n_x*n_y*n_theta*n_phi)
    for h=1:(n_v*n_w)
        if(Cont(i,h)==0)
            Delta2((i-1)*n_v*n_w+h,:)=0;
        end
    end
end
save('Delta2','Delta2')

 

disp('  Fixed point reached')
disp(' ')
whos Cont
toc
disp(' ')
%figure('Name','Controller','NumberTitle','off');
%plot_cont_HDV_pre(Cont,[n_x;n_x],[n_modes;n_modes],bound,d_x)

return
Cont1=Cont;
save('Controller_save','Cont1')

% Simulation

% x140=[449 449.4]';
% x230=[449.67 449.65]';
x230=[449.5641;
  449.6280];
x140=[449.7227;   449.7440];
x23=x230;
x14=x140;
traj=[];
traj1=[];
traj2=[];
trajd=[];
trajq=[];
noise=300; % noise
D0=[-1000;-500];
D1=[-5000;-500];
D2=[-5000;-4500];
Dnew=[-1800;-4500];

% D0=[-2500;-2500];
% D1=D0;
% D2=D0;

T_s=0.06; % simulation time
N_s=floor(T_s/tau); % iterations at simulation sampling
m=zeros(N_s,1);
pb=waitbar(0);
for i=1:N_s
    waitbar(i/N_s,pb,sprintf('Simulating - %2.2f',i/N_s))
    q=floor(([x23(1);x23(2)]-bound(:,1))./d_x)+[1;1];
    v=find(Cont((q(1)-1)*n_x+q(2),:));
    
 %%%%%%%%%%%%%%   TRIVIAL CONTROL
%     

        
    u_1=zeros(1,length(v));
    u_2=zeros(1,length(v));
    J=zeros(1,length(v));
    
    for j=1:length(v)
          a=ceil(v(j)/n_modes);
          resto=rem(v(j),n_modes);
          if resto==0
              b=n_modes;
          else
              b=resto;
          end
          u_1(j)=u_values1(a);
          u_2(j)=u_values2(b);
          J(j)=u_1(j)^2+u_2(j)^2;
          
    end
   
    
   [M,m]=min(J);
   u_values=[u_1(m);u_2(m)];
 
   n(1,1)=noise*(2*rand(1)-1);
   n(2,1)=noise*(2*rand(1)-1);

   if i<N_s/4 % First period
    x23=Four_Unit_Successor_x1(x23,u_values,x14,tau);
    x14=Four_Unit_Successor_x2(x14,x23,D0+n,tau);
    d=D0+n;
   elseif  i>=N_s/4 && i<=2/4*N_s % Second period
    x23=Four_Unit_Successor_x1(x23,u_values,x14,tau);
    x14=Four_Unit_Successor_x2(x14,x23,D1+n,tau);
    d=D1+n;
   
   elseif i>=2/4*N_s && i<=3/4*N_s % Second period
    x23=Four_Unit_Successor_x1(x23,u_values,x14,tau);
    x14=Four_Unit_Successor_x2(x14,x23,D2+n,tau);
    d=D2+n;
    
   else % Third period
    x23=Four_Unit_Successor_x1(x23,u_values,x14,tau);
    x14=Four_Unit_Successor_x2(x14,x23,Dnew+n,tau);
    d=Dnew+n;
     
   
   end
    p1(i)=u_values(1) ;
    p2(i)=u_values(2) ;
    x=[x23;x14];
    traj=[traj x];
    traj1=[traj1 x23];
    traj2=[traj2 x14];
    trajd=[trajd d];
    trajq=[trajq q];
end
close(pb);


figure();
subplot(3,1,1);
plot(1:1:N_s,traj); 

subplot(3,1,2);
plot(1:1:N_s,[p1]); % Power injected by the controller 2

subplot(3,1,3);
plot(1:1:N_s,[p2]); % Power injected by the controller 2


figure();
plot(1:1:N_s,[p1+p2;-trajd(1,:)-trajd(2,:)]); % Power injected by the controller 2

subplot(3,2,1);
plot(1:1:N_s,traj1); 
title(['Sources (x=',num2str(n_x),')'])
grid;

subplot(3,2,2);
plot(1:1:N_s,traj2); 
title(['Loads ',])
grid;

subplot(3,2,4);
plot(1:1:N_s,trajd(1,:)); % Power absorbed by the load 1
title(['Power 1',]);
grid;

subplot(3,2,6);
plot(1:1:N_s,trajd(2,:)); % Power absorbed by the load 4
title(['Power 4',]);
grid;

subplot(3,2,3);
plot(1:1:N_s,p1); % Power injected by the controller 2
title(['Power 2 (u=',num2str(n_modes),', ps=',num2str(ps),')']);
grid;

subplot(3,2,5);
plot(1:1:N_s,p2); % Power injected by the controller 3
title(['Power 3 (u=',num2str(n_modes),', ps=',num2str(ps),')']);
grid;

figure();
plot(1:1:N_s,trajq); 
title(['States - x=',num2str(n_x),', u=',num2str(n_modes),]);
grid on;