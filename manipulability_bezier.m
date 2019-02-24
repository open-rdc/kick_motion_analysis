clear;
clf;

%parameter
height = 0.19 % (m)
stroke = 0.10 % (m)
motor_velocity = 4.81; % (rad/s) 46rpm
l1 = 0.108; % (m) length of link
l2 = 0.108;
ratio = 0.1; % display
P = [0, 0; 0, 0.06; 0.17, 0.06; 0.1, 0.0];
period = 0.30;

function [t1, t2] = inverse_kinematics(x, y, l1, l2)
  l = sqrt(x^2+y^2);
  t1 = t2 = 0;
  if (l <= (l1+l2))
    t1 = atan2(y,x) + acos((l1^2+l^2-l2^2)/(2*l2*l));
    t2 = pi + acos((l1^2+l2^2-l^2)/(2*l1*l2)) + t1;
  endif
endfunction

function [x, y] = bezier(P, t)
  B =[(1-t)^3, 3*t*(1-t)^2, 3*t^2*(1-t), t^3];
  X = B * P;
  x = X(1);
  y = X(2);
endfunction

function [xd, yd] = bezier_d(P, t)
  Bd =[-3*(1-t)^2, 3*(1-t)^2-6*t*(1-t), 6*t*(1-t)-3*t^2, 3*t^2];
  Xd = Bd * P;
  xd = Xd(1);
  yd = Xd(2);
endfunction

v = [-1 -1 1 1 -1; -1 1 1 -1 -1];

i = 0;
for t = 0.0: 0.2: 1.0
  [xt, yt] = bezier(P, t);
  x = xt - stroke/2;
  y = yt - height;

  [t1, t2] = inverse_kinematics(x, y, l1, l2);
  if (t1 != 0 || t2 != 0)
    i = i + 1;
    x1 = l1 * cos(t1);
    y1 = l1 * sin(t1);
    x2 = l1 * cos(t2) + x1;
    y2 = l1 * sin(t2) + y1;
    arm_x(i,:) = [0 x1 x2];
    arm_y(i,:) = [0 y1 y2];
    jaco = [-l1*sin(t1) -l2*sin(t2); l1*cos(t1) l2*cos(t2)];
	m = jaco * v;
	mx(i,:) = m(1,:) * ratio + x;
	my(i,:) = m(2,:) * ratio + y;
	[xd, yd] = bezier_d(P, t);
    xdp(i,:) = [x x + xd / period / motor_velocity * ratio];
    ydp(i,:) = [y y + yd / period / motor_velocity * ratio];
  endif
endfor

hold on;
axis([-0.15,0.15,-0.25,0.05],"square");
for n = 1:i
  plot(arm_x(n,:), arm_y(n,:));
  plot(xdp(n,:), ydp(n,:), "r");
  plot(mx(n,:), my(n,:));
end
hold off;

