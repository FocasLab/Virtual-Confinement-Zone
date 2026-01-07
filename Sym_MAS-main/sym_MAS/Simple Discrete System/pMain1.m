function[] = pDiff_Successor(x,y,theta,v,w,n_x,n_y,n_theta,n_v,n_w)

bound_P=[-1000 0];
init=[bound(:,1)+[(x-1);(y-1);(theta-1)+(bound(3,1)/d_theta)].*d_states,...
    bound(:,1)+[(x);(y);theta+(bound(3,1)/d_theta)].*d_states];

% Calculate Successors of the extreme values of symbol
u = [v;w];

% Robot dynamics

xp1=diff_Successor(init(:,1),u,bound_P(:,1),tau);
xp2=diff_Successor(init(:,2),u,bound_P(:,2),tau);

% Check if succesors are within bounds of State space
if all(xp1 >= bound(:,1)) && all(xp2 <= bound(:,2))
    minsucc=floor((xp1-bound(:,1))./d_states)+[1;1;1]; % associate successors of extrema to extremal intervals
    maxsucc=ceil((xp2-bound(:,1))./d_states);
    for ip1=minsucc(1):maxsucc(1) % compute the whole interval contained into the extremal intervals
        for ip2=minsucc(2):maxsucc(2)
            for ip3=minsucc(3):maxsucc(3)
                Delta((x-1)*n_y*n_theta*n_v*n_w+(y-1)*(n_theta)*n_v*n_w+(theta-1)*n_v*n_w+(v-1)*n_w+w,(ip1-1)*n_y*n_theta+(ip2-1)*n_theta+ip3)=1;  %
            end
        end
    end
end