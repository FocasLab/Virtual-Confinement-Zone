function Succ = diff_Successor(V,u,P,time_step)

kinematicModel = differentialDriveKinematics; % Import dynamics of differential drive system
[t,y] = ode23tb(@(t,V)derivative(kinematicModel, V, u),[0 time_step],V);
Succ=y(end,:)'; % [x;y;theta]
 
end 
