function H = forward_kuka(joint, myrobot)
% initialize Anonymous Function H
H_templete = @(theta,alpha,a,d)[cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta);
    sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);
    0 sin(alpha) cos(alpha) d;
    0 0 0 1];
	

theta1=joint(1);
theta2=joint(2);
theta3=joint(3);
theta4=joint(4);
theta5=joint(5);
theta6=joint(6);

alpha1=myrobot.alpha(1);
alpha2=myrobot.alpha(2);
alpha3=myrobot.alpha(3);
alpha4=myrobot.alpha(4);
alpha5=myrobot.alpha(5);
alpha6=myrobot.alpha(6);

a1=myrobot.a(1);
a2=myrobot.a(2);
a3=myrobot.a(3);
a4=myrobot.a(4);
a5=myrobot.a(5);
a6=myrobot.a(6);

d1=myrobot.d(1);
d2=myrobot.d(2);
d3=myrobot.d(3);
d4=myrobot.d(4);
d5=myrobot.d(5);
d6=myrobot.d(6);


% compute intermeida Hs
H1= H_templete(theta1,alpha1,a1,d1);
H2= H_templete(theta2,alpha2,a2,d2);
H3= H_templete(theta3,alpha3,a3,d3);
H4= H_templete(theta4,alpha4,a4,d4);
H5= H_templete(theta5,alpha5,a5,d5);
H6= H_templete(theta6,alpha6,a6,d6);



%To find a point in the 6th frame in reference to the base frame we do
%multiply the Homogenous matrices
H=H1*H2*H3*H4*H5*H6




%H = H01*H12*H23*H34*H45*H56; 
end