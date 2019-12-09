function [qref, way] = motionplan(q0,q2,t1,t2,myrobot,obs,tol, alpha, beta)
    %initialize q, waypoint and iteration count
    qi = q0;
    j = 0;
    way = [];
    way(1,:) = qi;
    %set learning rates, alpha for Fatt and beta for Frep
    %alpha = 0.013;
    %beta = 0.01;
    %iterate until requirement is met
    while norm(q2(1:5)-qi(1:5)) >= tol
        %update with Fatt
        qi = qi + alpha * att(qi,q2,myrobot)/norm(att(qi,q2,myrobot));
        %update with Frep for each obstacle if they have affect
        for i = 1:numel(obs)
            Frep = norm(rep(qi,myrobot,obs{i}));
            if Frep > 0
                qi = qi + beta * rep(qi,myrobot,obs{i})/norm(rep(qi,myrobot,obs{i}));
            end
        end
        %update way points and iteration number
        way = [way; qi];
        %for monitoring progress
        if mod(j,100) == 0
            disp(j)
            disp(norm(q2(1:5)-qi(1:5)))
        end
        j = j + 1;
    end
    way(:,6) = linspace(q0(6),q2(6),size(way,1));
    t = linspace(t1,t2,size(way,1));
    qref =  spline(t,way');
end