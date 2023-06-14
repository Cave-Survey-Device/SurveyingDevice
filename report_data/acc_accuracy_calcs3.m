x4 = linspace(0, 2*pi,1000);
y41 = sin(x4); 
y42 = cos(x4);
x4_plot = rad2deg(x4);
plot(x4_plot,y41,Color='red');
hold on;
plot(x4_plot,y42,Color='blue');