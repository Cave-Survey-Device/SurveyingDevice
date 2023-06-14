samples = importdata("accel_data.txt");
% samples = samples(:,2:7);
g = samples(:,1:3);
gx = g(:,1);
gy = g(:,2);
gz = g(:,3);



m = samples(:,4:6);
% m = (roty(68)*m')';
mx = m(:,1);
my = m(:,2);
mz = m(:,3);
% mz = sign(mz).*(abs(mz)-1);




gnorm = vecnorm(g,2,2);
mnorm = vecnorm(m,2,2);


inclination1 = rad2deg(asin(gx./gnorm));
inclination2 = rad2deg(atan(gx./sqrt(gy.^2+gz.^2)));
z_tilt = rad2deg(acos(gz./gnorm));
% roll = rad2deg(asin(gy./gnorm));

% roll is the remaining angle after reversing the inclination
inc_rev = g;
for i = 1:size(inclination)
    inc_rev(i,:) = (roty(-inclination(i)) * g(i,:)')';
end

roll1 = rad2deg(atan(inc_rev(:,2)./inc_rev(:,3)));
roll2 = rad2deg(atan(gy/sqrt(gx.^2+gz.^2)));
roll3 = sqrt(z_tilt.^2 - inclination.^2);


% roll = sqrt(z_tilt.^2 - inclination.^2);
heading = zeros(size(inclination));

figure();
for i = 1:size(inclination)
    mn = roty(inclination(i)) * m(i,:)';
    heading(i) = rad2deg(atan2(mn(2),mn(1)));
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

plot(inclination1,Color="red");
hold on;
plot(inclination2,Color="cyan");
hold on;

plot(roll1,Color="green");
plot(roll3,Color="black");
legend("heading", "inclination1", "inclination2", "roll1", "roll2")
