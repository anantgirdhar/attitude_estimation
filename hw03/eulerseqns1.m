function dydt = eulerseqns1(t,y)
% AE 6356 Spacecraft Attitude
% Assignment 3 Problem 1
% Glenn Lightsey Fall 2024
%
% define mass properties
m=4;     % mass in kg
h=0.34;   % height in m  (x)
w=0.1;   % width in m (y)
d=0.1;  % depth in m (z)
%
% rectangular prism spacecraft model
Jx=m/12*(w^2+d^2);
Jy=m/12*(h^2+d^2);
Jz=m/12*(h^2+w^2);
%
% principal axis inertia tensor
J=[Jx,0,0;0,Jy,0;0,0,Jz];
%
% calculate coefficients
c1=(Jy-Jz)/Jx; 
c2=(Jz-Jx)/Jy; 
c3=(Jx-Jy)/Jz; 
%
w1=y(1); w2=y(2); w3=y(3);
w=[w1;w2;w3];
q1=y(4); q2= y(5); q3=y(6); q4=y(7);
xi=[q4,-q3,q2;q3,q4,-q1;-q2,q1,q4;-q1,-q2,-q3];
qd=0.5*xi*w;

dydt = [    c1*w2*w3 
            c2*w3*w1 
            c3*w1*w2
            qd(1)
            qd(2)
            qd(3)
            qd(4)];

