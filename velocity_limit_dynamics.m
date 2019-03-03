clear;
clf;

%parameter
height = 0.19 % (m)
stroke = 0.20 % (m)
motor_velocity = 4.81 % (rad/s) 46rpm
l1 = 0.108 % (m) length of link
l2 = 0.108
ratio = 0.05; % display
P = [0, 0; 0, 0.06; stroke+0.2, 0.06; stroke, 0.0];
period = 0.62 % (s)
cog = 0.3 % (m) center of gravity
m1 = 0.23+0.2;
m2 = 0.35+0.3;
Im = 0.0;
I1 = 1/4 * m1 * l1^2;
I2 = 1/4 * m2 * l2^2;
p_gain = 50.0;
d_gain = 0.5;
max_torque = 4.6; % (Nm)

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

function [t1, t2, t1d, t2d] = dynamics(t1, t2, t1d, t2d, f1, f2, l1, l2, m1, m2, I1, I2, Im, motor_velocity, dt)
  g = 9.8;
  M = [I1+l1^2*m2+l1^2*m1+Im l1*l2*m2*cos(t2-t1);
       l1*l2*m2*cos(t2-t1) (2*l2^2*m2+2*I2)/2+Im];
  D = [(l1*l2*m2*t1d*t2d-l1*l2*m2*t2d^2)*sin(t2-t1);
       (l1*l2*m2*t1d^2-l1*l2*m2*t1d*t2d)*sin(t2-t1)];
  G = [g*l1*m2*cos(t1)+g*l1*m1*cos(t1);
       g*l2*m2*cos(t2)];
  tdd = inv(M) * [[f1; f2] - D - G];
  t1dd = tdd(1);
  t2dd = tdd(2);
  t1d = max([min([t1d + t1dd * dt, motor_velocity]), -motor_velocity]);
  t2d = max([min([t2d + t2dd * dt, motor_velocity]), -motor_velocity]);
  t1 = t1 + t1d * dt + 1 / 2 * t1dd * dt^2;
  t2 = t2 + t2d * dt + 1 / 2 * t2dd * dt^2;
endfunction

g = 9.8;
Tc = sqrt(cog/g)
x0 = -stroke/4;
v0 = stroke/4*(cosh(period/Tc)+1)/(Tc*sinh(period/Tc))

xr = -stroke/4;
yr = -height;
[t1r, t2r] = inverse_kinematics(xr, yr, l1, l2);
t1rd = 0; t2rd = 0;
jaco = [-l1*sin(t1r) -l2*sin(t2r); l1*cos(t1r) l2*cos(t2r)];
trd = inv(jaco) * [-v0; 0];
t1rd = trd(1)
t2rd = trd(2)

i = 0;
dt = 0.001;

for t = 0.0: dt/period: 1 * period/period
%  sup_pos = x0 * cosh(mod(t,1) * period / Tc) + Tc * v0 * sinh(mod(t,1) * period / Tc) - x0;
  sup_pos = x0 * cosh(t * period / Tc) + Tc * v0 * sinh(t * period / Tc) - x0;
  if (t <= 1)
    [xt, yt] = bezier(P, t);
    x = xt - stroke/4;
    y = yt - height;
  else
    x = stroke*3/4-sup_pos;
  endif

  if (t <= 1)
    sup_leg_pos = sup_pos;
  else
    sup_leg_pos = stroke/2;
  endif
  [t1, t2] = inverse_kinematics(x - sup_leg_pos, y, l1, l2);
  w1r = -t1rd/motor_velocity;
  f1 = max([min([p_gain * (t1 - t1r) - d_gain * t1rd, max_torque * (1 + w1r)]), -max_torque * (1 - w1r)]);
  w2r = -t2rd/motor_velocity;
  f2 = max([min([p_gain * (t2 - t2r) - d_gain * t2rd, max_torque * (1 + w2r)]), -max_torque * (1 - w2r)]);
  [t1r, t2r, t1rd, t2rd] = dynamics(t1r, t2r, t1rd, t2rd, f1, f2, l1, l2, m1, m2, I1, I2, Im, motor_velocity, dt);
  tor = [w1r-1 w1r-1 w1r+1 w1r+1 w1r-1; w2r-1 w2r+1 w2r+1 w2r-1 w2r-1];

%  if (t1 != 0 || t2 != 0)
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
%    printf("%f %f %f %f %f %f %f %f\n", x2r, y2r, t1, t2, t1r, t2r, t1rd, t2rd);
%  endif
endfor

hold on;
axis([-0.10,0.25,-0.25,0.10],"square");
for n = 1:i
  if (mod(n - 1, 50) == 0)
    plot(arm_x(n,:), arm_y(n,:));
    plot(arm_xr(n,:), arm_yr(n,:), "r");
    plot(mx(n,:), my(n,:), "r");
    plot(px(n), py(n), ".or");
  endif
  plot(xdp, ydp);
  plot(xdpr, ydpr, "r");
end
hold off;

