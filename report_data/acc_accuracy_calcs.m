alpha = 0.25;

x1 = linspace(-90,90,1000);
y11 = sin((x1+alpha)*pi/180) - sin(x1*pi/180);
y12 = abs(cos((x1+alpha)*pi/180) - cos(x1*pi/180));

x2 = linspace(-90,-80,1000);
y2 = sin((x2+alpha)*pi/180) - sin(x2*pi/180);

x3 = linspace(90,80,1000);
y3 = sin(x3*pi/180);


% subplot(2, 3, [1 2 4 5])   %2 x 2 to the left
plot(x1,y11);
hold on;
plot(x1,y12,Color="red");
xlabel('Inclination angle [degrees]')
ylabel('Incrimental sensitivity [LSB/g]')
title(['Incrimental Sensitivity VS Inclination Angle for 0.25' char(176) ' sensitivity']);
yline(3.0853e-3,LineStyle="--");
legend("sin(x)", "cos(x)","Decision margin");


% subplot(2, 3, 3)           %1 x 1 top right
% plot(x2,y2);
% xlabel('Inclination angle [degrees]')
% ylabel('Incrimental sensitivity [LSB/g]')
% 
% 
% subplot(2, 3, 6)           %1 x 1 bottom right
% plot(x3,y3);
% xlabel('Inclination angle [degrees]')
% ylabel('Incrimental sensitivity [LSB/g]')

