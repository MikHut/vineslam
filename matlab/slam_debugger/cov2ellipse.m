clear;
close all;

% robot position and landmark observation
r = [2, 2];
z = [3,0];
l = [(r(1) + z(1)*cos(z(2))), (r(2) + z(1)*sin(z(2)))];

k = 0.5;

% covariance initialization
P = zeros(2,2);
P(1,1) = ((z(1)^2 / (0.12 * 500)) * 0.2);
P(2,2) = 0.01 / (z(1)^2);
P(1,2) = 0;
P(2,1) = 0;

figure(2)
hold on
x = 0.2:0.1:5;
plot(x,x.^2 / ((0.12 * 500)) * 1);
plot(x,(x.^2 / ((0.12 * 500)) * 1) * k);

R = [cos(z(2)), -sin(z(2));
     sin(z(2)), +cos(z(2))];

P = R*P*R';
     
% eigen decomposition of matrix P
a = P(1,1);
b = P(1,2);
c = P(2,2);

lambda_1 = (a + c) / 2 + sqrt(((a-c)/2)^2 + b^2);
lambda_2 = (a + c) / 2 - sqrt(((a-c)/2)^2 + b^2);

if b == 0 && a >= c 
    th = 0;
elseif b == 0 && a < c
    th = pi/2;
else
    th = atan2(lambda_1 - a, b);
end

std_x = sqrt(lambda_1);
std_y = sqrt(lambda_2);

% draw ellipse
t = linspace(0,2*pi);
x = l(1) + std_x * cos(th) * cos(t) - std_y * sin(th) * sin(t);
y = l(2) + std_x * sin(th) * cos(t) + std_y * cos(th) * sin(t);

figure(1);
hold on;
grid on;
axis equal;

plot(r(1), r(2), 'x');
plot([r(1), l(1)], [r(2), l(2)], 'g');
plot(x, y, 'r');