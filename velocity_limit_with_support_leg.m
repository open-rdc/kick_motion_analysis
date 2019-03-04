clear;
clf;

%parameter
height = 0.198 % (m)
stroke = 0.20 % (m)
motor_velocity = 4.81 % (rad/s) 46rpm
l1 = 0.108 % (m) length of link
l2 = 0.108
ratio = 0.05; % display
P = [0, 0; 0, 0.06; stroke+0.2, 0.06; stroke, 0.0];
period = 0.31 % (s)
cog = 0.3 % (m) center of gravity

%parameter low
%stroke = 0.10 % (m)
%P = [0, 0; 0, 0.06; 0.17, 0.06; stroke, 0.0];

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

xr = -stroke/4;
yr = -height;
[t1r, t2r] = inverse_kinematics(xr, yr, l1, l2);

i = 0;
dt = 0.01;
g = 9.8;
Tc = sqrt(cog/g)
x0 = -stroke/4;
v0 = stroke/4*(cosh(period/Tc)+1)/(Tc*sinh(period/Tc))

for t = 0.0: dt/period: period/period
  [xt, yt] = bezier(P, t);
  x = xt - stroke/4;
  y = yt - height;

  sup_leg_pos = x0 * cosh(t * period / Tc) + Tc * v0 * sinh(t * period / Tc) - x0;
  [t1, t2] = inverse_kinematics(x - sup_leg_pos, y, l1, l2);
  t1r = t1r + max([min([t1 - t1r, motor_velocity * dt]), -motor_velocity * dt]);
  w1r = -max([min([(t1 - t1r) / dt, motor_velocity]), -motor_velocity])/motor_velocity;
  t2r = t2r + max([min([t2 - t2r, motor_velocity * dt]), -motor_velocity * dt]);
  w2r = -max([min([(t2 - t2r) / dt, motor_velocity]), -motor_velocity])/motor_velocity;
  tor = [w1r-1 w1r-1 w1r+1 w1r+1 w1r-1; w2r-1 w2r+1 w2r+1 w2r-1 w2r-1];

  if (t1 != 0 || t2 != 0)
    i = i + 1;
    x1 = l1 * cos(t1);
    y1 = l1 * sin(t1);
    x2 = l1 * cos(t2) + x1;
    y2 = l1 * sin(t2) + y1;
    arm_x(i,:) = [sup_leg_pos x1 + sup_leg_pos x2 + sup_leg_pos];
    arm_y(i,:) = [0 y1 y2];
    xdp(i) = x2 + sup_leg_pos;
    ydp(i) = y2;
    x1r = l1 * cos(t1r);
    y1r = l1 * sin(t1r);
    x2r = l2 * cos(t2r) + x1r;
    y2r = l2 * sin(t2r) + y1r;
    arm_xr(i,:) = [sup_leg_pos x1r+sup_leg_pos x2r+sup_leg_pos];
    arm_yr(i,:) = [0 y1r y2r];
    xdpr(i) = x2r + sup_leg_pos;
    ydpr(i) = y2r;
    jaco = [-l1*sin(t1) -l2*sin(t2); l1*cos(t1) l2*cos(t2)];
	m = jaco' * tor;
	mx(i,:) = m(1,:) * ratio + x2r + sup_leg_pos;
	my(i,:) = m(2,:) * ratio + y2r;
    px(i) = x2r + sup_leg_pos;
    py(i) = y2r;
  endif
endfor

hold on;
axis([-0.10,0.25,-0.25,0.10],"square");
for n = 1:i
  if (mod(n - 1, 3) == 0)
    plot(arm_x(n,:), arm_y(n,:));
    plot(arm_xr(n,:), arm_yr(n,:), "r");
    plot(mx(n,:), my(n,:), "r");
    plot(px(n), py(n), ".or");
  endif
  plot(xdp, ydp);
  plot(xdpr, ydpr, "r");
end
hold off;

