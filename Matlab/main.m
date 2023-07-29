FILE_mag_real_samples =       'mag_samples.txt';
FILE_mag_real_corrections =   'mag_corrections.txt'; 
FILE_mag_test_samples =       'test_mag_samples.txt';
FILE_mag_test_corrections =   'test_mag_corrections.txt'; 

plot_data(FILE_mag_real_samples, FILE_mag_real_corrections, ...
    FILE_mag_test_samples, FILE_mag_test_corrections, "Magnetometer");

FILE_acc_real_samples =       'acc_samples.txt';
FILE_acc_real_corrections =   'acc_corrections.txt'; 
FILE_acc_test_samples =       'test_acc_samples.txt';
FILE_acc_test_corrections =   'test_acc_corrections.txt'; 

plot_data(FILE_acc_real_samples, FILE_acc_real_corrections, ...
    FILE_acc_test_samples, FILE_acc_test_corrections, "Accelerometer");