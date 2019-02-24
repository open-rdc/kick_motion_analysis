clear;
clf;

%parameter
height = 0.19 % (m)
stroke = 0.20 % (m)
motor_velocity = 4.81; % (rad/s) 46rpm
l1 = 0.108; % (m) length of link
l2 = 0.108;
ratio = 0.1; % display
P = [0, 0; 0, 0.1; 0.3, 0.1; stroke, 0.0];
period = 0.30; % (s)
cog = 0.3 % (m) center of gravity

function [t1, t2] = inverse_kinematics(x, y, l1, l2)
  l = sqrt(x^2+y^2);
  t1 = t2 = 0;
  if (l <= (l1+l2))
    t1 = atan2(y,x) + acos((l1^2+l^2-l2^2)/(2*l2*l));
    t2 = pi + acos((l1^2+l2^2-l^2)/(2*l1*l2)) + t1;
  endif
endfunction

xr = stroke/4;
yr = -height;
[t1r, t2r] = inverse_kinematics(xr, yr, l1, l2);

i = 0;
dt = 0.01;
g = 9.8;
Tc = sqrt(cog/g)
x0 = -stroke/4;
v0 = stroke/4*(cosh(period/Tc)+1)/(Tc*sinh(period/Tc))

for t = 0.0: dt/period: period/period
  sup_leg_pos = x0 * cosh(t * period / Tc) + Tc * v0 * sinh(t * period / Tc) - x0
  x = - sup_leg_pos + stroke/4;
  y = - height;

  [t1, t2] = inverse_kinematics(x, y, l1, l2);
  t1r = t1r + max([min([t1 - t1r, motor_velocity * dt]), -motor_velocity * dt]);
  t2r = t2r + max([min([t2 - t2r, motor_velocity * dt]), -motor_velocity * dt]);

  if (t1 != 0 || t2 != 0)
    i = i + 1;
    x1 = l1 * cos(t1);
    y1 = l1 * sin(t1);
    x2 = l1 * cos(t2) + x1;
    y2 = l1 * sin(t2) + y1;
    arm_x(i,:) = [0 x1 x2];
    arm_y(i,:) = [0 y1 y2];
    xdp(i) = x2;
    ydp(i) = y2;
    x1r = l1 * cos(t1r);
    y1r = l1 * sin(t1r);
    x2r = l2 * cos(t2r) + x1r;
    y2r = l2 * sin(t2r) + y1r;
    arm_xr(i,:) = [0 x1r x2r];
    arm_yr(i,:) = [0 y1r y2r];
    xdpr(i) = x2r;
    ydpr(i) = y2r;
  endif
endfor

hold on;
axis([-0.15,0.15,-0.25,0.05],"square");
for n = 1:i
  if (mod(n - 1, 5) == 0)
    plot(arm_x(n,:), arm_y(n,:));
    plot(arm_xr(n,:), arm_yr(n,:), "r");
  endif
  plot(xdp, ydp);
  plot(xdpr, ydpr, "r");
end
hold off;

