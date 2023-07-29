% Plotter to compare simulated and real data
% Data must be stored in a CSV of Nx3

% Configure sources
function plot_data( FILE_real_samples, FILE_real_corrections, ...
                    FILE_test_samples, FILE_test_corrections, name)
    
    % Import data
    real_samples =      importdata(FILE_real_samples);
    real_corrections =  importdata(FILE_real_corrections);
    test_samples =      importdata(FILE_test_samples);
    test_corrections =  importdata(FILE_test_corrections);
    
    % Calculate normal vector of data
    real_sam_norm =     vecnorm(real_samples');
    real_corr_norm =    vecnorm(real_corrections');
    test_sam_norm =     vecnorm(test_samples');
    test_corr_norm =    vecnorm(test_corrections');
    
    % Plot data
    figure;
    plot(0:1:59,real_sam_norm,'r');
    hold on
    plot(0:1:59,real_corr_norm,'g');
    legend(name + ' samples','Corrected ' + name + ' data')
    title("Sensor data norm")
    ylim([0 1.25])
    
    figure;
    plot(0:1:59,test_sam_norm,'r');
    hold on
    plot(0:1:59,test_corr_norm,'g');
    legend('Test ' + name + ' samples','Corrected test ' + name + ' data')
    title("Test data norm")
    ylim([0 1.25])
end