function tau = rep(q,myrobot,obs)
% initialize the attractive force part of gradient descent
tau = zeros(1,6);
% predefine the variable to save the previous values
z_mem = zeros(3,7);
z_mem(3,1) = 1;
o_mem = zeros(3,7);

% compute the tau for each joint
for i = 1:6
    %get H for this joint
    H = myrobot.A(1:i,q(1:i)).T();
    
    % compute the Jacobian matrix
    J = zeros(3,6);
    for j = 1 : i
        J(:,j) = cross(z_mem(:,j), H(1:3,4)-o_mem(:,j));
    end
    
    % Compute repulsive force
    if strcmp(obs.type,'cyl')
        if H(3,4) <= obs.h
            n = max(norm(H(1:2,4)-obs.c)-obs.R, 0);
            o = (norm(H(1:2,4)-obs.c) - obs.R)/norm(H(1:2,4)-obs.c) * ...
                [H(1,4)-obs.c(1);H(2,4)-obs.c(2);0];
        elseif (H(3,4) > obs.h)&&(norm(H(1:2,4)-obs.c)>obs.R)
            n = max(sqrt((norm(H(1:2,4)-obs.c)-obs.R)^2 + (H(3,4)-obs.h)^2), 0);
            o = (norm(H(1:2,4)-obs.c) - obs.R)/norm(H(1:2,4)-obs.c) * ...
                [H(1,4)-obs.c(1);H(2,4)-obs.c(2);0] + [0;0;H(3,4)-obs.h];
        else
            n = max(H(3,4)-obs.h, 0);
            o = [0;0;H(3,4)-obs.h];
        end
    else
        n = max(H(3,4)-32, 0);
        o = [0;0;H(3,4)-32];
    end
    
    if n <= obs.rho0
        F_repulsive = (1/n - 1/obs.rho0)*(1/n^2)*(o/n);
    else
        F_repulsive = [0;0;0];
    end
    %calculate update
    tau = tau + F_repulsive'*J;
    %udpate memory
    z_mem(:,i+1) = H(1:3,3);
    o_mem(:,i+1) = H(1:3,4);
end
% normalize the output
tau = tau/norm(tau);
end