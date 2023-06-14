samples = importdata("accel_data.txt");
% samples = samples(:,2:7);
g = samples(:,1:3);
gx = g(:,1);
gy = g(:,2);
gz = g(:,3);

m = samples(:,4:6);
% m = (roty(-68)*m')';
mx = m(:,1);
my = m(:,2);
mz = m(:,3);
% mz = sign(mz).*(abs(mz)-1);

gnorm = vecnorm(g,2,2);
mnorm = vecnorm(m,2,2);
z_tilt = rad2deg(acos(gz./gnorm));


inclination = rad2deg(asin(gx./gnorm));
roll = rad2deg(atan(gy./gz));
roll2 = sqrt(z_tilt.^2-inclination.^2);

% roll = sqrt(z_tilt.^2 - inclination.^2);
heading = zeros(size(inclination));
heading2 = heading;
figure();
for i = 1:size(inclination)
    mn =  rotx(roll(i)) * roty(inclination(i)) * m(i,:)';
    heading(i) = rad2deg(atan2(mn(2),mn(1)));

    mn =  rotx(roll(i)) * m(i,:)';
    heading2(i) = rad2deg(atan2(mn(2),mn(1)));
    % scatter3(mn(1),mn(2),mn(3));
    % hold on;
end

hold on;
plot3(mx,my,mz,Color="blue");
hold on;
plot3(gx,gy,gz,Color="red");
hold on;
plot3(inc_rev(:,1),inc_rev(:,2),inc_rev(:,3),Color="black");
legend("Magnetometer", "Accelerometer","Projection")


figure();
plot(heading,Color="blue");
hold on;
plot(heading2,Color="cyan");
hold on;
plot(inclination,Color="red");
hold on;

plot(roll,Color="green");
legend("heading1", "heading2", "inclination", "roll")
