%copied from lab2.m, setting up DH table for mypuma560
DH = [pi/2 25 0 400;
      0 315 0 0;
      pi/2 35 0 0;
     -pi/2 0 0 365;
      pi/2 0 0 0;
      0 -156 0 161.44];
DH_forces =  [pi/2 25 0 400;
              0 315 0 0;
              pi/2 35 0 0;
             -pi/2 0 0 365;
              pi/2 0 0 0;
              0 0 0 161.44];
% initialize myrobot
kuka = mykuka(DH);
kuka_forces = mykuka(DH_forces);

setupobstacle_lab4prep;
tau = rep([pi/10,pi/12,pi/6,pi/2,pi/2,-pi/6],kuka_forces,prepobs{1})



%{
p1 = [620 375 50];
p2 = [620 -375 50];
R=[0 0 1;0 -1 0;1 0 0];
H1=[R p1';zeros(1,3) 1];
H2=[R p2';zeros(1,3) 1];
q1 = inverse_kuka(H1, kuka);
q2 = inverse_kuka(H2, kuka);

qref = motionplan(q1,q2,0,10,kuka_forces,prepobs,0.1);
t=linspace(0,10,300);
q = ppval(qref,t)';
plot(kuka,q)
%}
%Actual lab

%define four points
z_grid= 45;
p0 = [370 -440 150];
p1 = [370 -440 z_grid];
p2 = [750 -220 225];
p3 = [620 350 225];
p4 = [620 0 750];
R0 = [0 0 1; 0 -1 0; 1 0 0];
H0=[R0 p0';zeros(1,3) 1];
H1=[R0 p1';zeros(1,3) 1];
H2=[R0 p2';zeros(1,3) 1];
H3=[R0 p3';zeros(1,3) 1]; 
H4=[R0 p4';zeros(1,3) 1];

qh = [0 1.5708 0 0 1.5708 0]; 
q0 = inverse_kuka(H0, kuka);
q1 = inverse_kuka(H1, kuka);
q2 = inverse_kuka(H2, kuka);
%q3 = inverse_kuka(H3, kuka);
%for creative
q3 = inverse_kuka(H4, kuka);

setupobstacle()
[qrefh0, wayh0] = motionplan(qh,q0,0,10,kuka_forces,obs,0.01, 0.01, 0.015);
disp('done0')
[qref01, way01] = motionplan(q0,q1,0,10,kuka_forces,obs,0.01, 0.01, 0.015);
disp('done1')
[qref12, way12] = motionplan(q1,q2,0,10,kuka_forces,obs,0.01, 0.01, 0.015);
disp('done2')
[qref23, way23] = motionplan(q2,q3,0,10,kuka_forces,obs,0.01, 0.01, 0.015);
disp('done3')

hold on;
axis([-1500 1500 -1500 1500 0 1000]);
view(50,100);
plotobstacle(obs);
plot(kuka, [wayh0(1:5:end,:);way01(1:5:end,:);way12(1:5:end,:);way23(1:5:end,:)]);
hold off;

%{
for i=1:size(wayh0,1)
     q = way01(:,i)
     setAngles(q,0.04)
end
for i=1:size(way01,1)
     q = way01(:,i)
     setAngles(q,0.04)
end
 for i=1:size(way12,1)
     q = way12(:,i)
     setAngles(q,0.04)
 end
 for i=1:size(way23,1)
     q = way23(:,i)
     setAngles(q,0.04)
 end
%}



