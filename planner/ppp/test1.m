clear all;
clc;


xp0 = 0;
xv0 = 1;
xa0 = 0;

xpf = 3;
xvf = 0;
xaf = 0;

yp0 = 0;
yv0 = 1;
ya0 = 0;

ypf = 3;
yvf = 0;
yaf = 0;

tf =  4;


t = 0:0.1:5


tic
D = ppp_points(yp0,yv0,ya0,ypf,yvf,yaf,tf);
toc
D = flipud(D); %ppp outputs [c5;c4...c0]
y = @(t) D(1) + D(2)*t + D(3)*t.^2 + D(4)*t.^3 + D(5)*t.^4 + D(6)*t.^5;

C = ppp_points(xp0,xv0,xa0,xpf,xvf,xaf,tf);
C = flipud(C); %ppp outputs [c5;c4...c0]
x = @(t) C(1) + C(2)*t + C(3)*t.^2 + C(4)*t.^3 + C(5)*t.^4 + C(6)*t.^5;

hold on;
%plot(t, x(t));
%plot(t, y(t));
plot(x(t),y(t));
axis([0 5 0 5]);
