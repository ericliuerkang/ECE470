
function tau = att(q,q2,myrobot)
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
    H_desired = myrobot.A(1:i,q2(1:i)).T();
    % compute the Jacobian matrix
    J = zeros(3,6);
    for j = 1 : i
        J(:,j) = cross(z_mem(:,j), H(1:3,4)-o_mem(:,j));
    end
    % Compute attractive force
    F_attractive = (H_desired(1:3,4) - H(1:3,4));
    %calculate update
    tau = tau + F_attractive'*J;
    %udpate memory
    z_mem(:,i+1) = H(1:3,3);
    o_mem(:,i+1) = H(1:3,4);
end
% normalize the output
tau = tau/norm(tau);
end