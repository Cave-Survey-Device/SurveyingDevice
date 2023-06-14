alpha = 0.001
angdeg = 88;
ang = deg2rad(angdeg);

z1 = sin(ang);
z2 = sin(ang) - alpha;

x1 = asin(z1);
x2 = asin(z2);

z3 = cos(x1);
z4 = cos(x2);

xrange = [rad2deg(x2)*0.99, rad2deg(x1)*1.01];


x4 = linspace(x2*0.99, x1*1.01,1000);
y41 = sin(x4); 
y42 = cos(x4);
x4_plot = rad2deg(x4);

figure();
subplot(2, 1, 1)
plot(x4_plot,y41,Color='red');
xlabel("Inclination angle");
ylabel("Measured g_x");
title("Measured x component of gravitational vector \Deltaz = 0.001 = 0.1%")
yline(z1);
yline(z2);
xline(rad2deg(x1));
xline(rad2deg(x2));
xlim(xrange);
ylim([z1-0.02, 1+0.02])


subplot(2, 1, 2)
plot(x4_plot,y42,Color='blue');
xlabel("Inclination angle");
ylabel("Calculated \theta_z");
title("Calculated z component of gravitational vector \Deltaz = 0.0218 = 2.2%")
yline(z3);
yline(z4);
xline(rad2deg(x1));
xline(rad2deg(x2));
xlim(xrange);
ylim([z3-0.01, z3 + 0.03])




