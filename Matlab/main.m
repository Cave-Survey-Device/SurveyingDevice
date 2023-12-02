FILE_mag_real_samples =       'mag_samples.txt';
FILE_mag_real_corrections =   'mag_corrections.txt'; 
% FILE_mag_test_samples =       'test_mag_samples.txt';
% FILE_mag_test_corrections =   'test_mag_corrections.txt'; 

FILE_mag_test_samples =       'test_data/sample_mag_data';
FILE_mag_test_corrections =   'test_data/mag_corrections'; 
% 
% FILE_acc_test_samples =       'test_data/sample_acc_data';
% FILE_acc_test_corrections =   'test_data/acc_corrections'; 

plot_data(FILE_mag_real_samples, FILE_mag_real_corrections, "Real Magnetometer");
plot_data(FILE_mag_test_samples, FILE_mag_test_corrections, "Test Magnetometer");

plot_3d_data(FILE_mag_real_samples, FILE_mag_real_corrections, "Real Magnetometer");
