% Plotter to compare simulated and real data
% Data must be stored in a CSV of Nx3

% Configure sources
FILE_test_data =   'laser_sample_data.txt'; 
test_data =      importdata(FILE_test_data);

x = test_data(1,:);
y = test_data(2,:);
z = test_data(3,:);




figure;
scatter3(x,y,z);
hold on;
scatter3(1,1,2);