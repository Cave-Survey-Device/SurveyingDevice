samples = importdata("accel_data.txt");
% samples = samples(:,2:7);
gx = samples(:,1);
gy = samples(:,2);
gz = samples(:,3);
g = samples(:,1:3);

mx = samples(:,4);
my = samples(:,5);
mz = samples(:,6);
m = samples(:,4:6);

gnorm = vecnorm(g,2,2);


device_xy_plane = [0 0 1];
device_xz_plane = [0 1 0];

device_xy_plane = repmat(device_xy_plane,20,1);
device_xz_plane = repmat(device_xz_plane,20,1);

true_xz_plane = cross(g,m);
true_xy_plane = g;

hea_proj = cross(true_xy_plane,device_xz_plane);
heading2 = rad2deg(atan2(hea_proj(:,3),hea_proj(:,1)));

hea_proj = cross(true_xz_plane,device_xz_plane);
heading3 = rad2deg(atan(hea_proj(:,3)./hea_proj(:,1)));
% heading3 = rad2deg(atan2(hea_proj(:,3),hea_proj(:,1)));

inclination = rad2deg(asin(gx./gnorm));


figure();
plot3(gx,gy,gz);

count = linspace(0,1,20);
figure();
plot(count,inclination);
hold on;
plot(count, heading2,Color="red");
plot(count, heading3,Color="green");

figure();
plot(count,sin(deg2rad(inclination)));
hold on;
plot(count,sin(deg2rad(heading2)));
plot(count,sin(deg2rad(heading3)));
