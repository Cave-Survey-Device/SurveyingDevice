% Plotter to compare simulated and real data
% Data must be stored in a CSV of Nx3

% Configure sources
function plot_data( FILE_samples, FILE_corrections, name)
    
    % Import data
    samples =      importdata(FILE_samples);
    corrections =  importdata(FILE_corrections);
    
    % Calculate normal vector of data
    if size(samples,2) > size(samples,1)
        sam_norm =     vecnorm(samples);
        corr_norm =    vecnorm(corrections);
    else
        sam_norm =     vecnorm(samples');
        corr_norm =    vecnorm(corrections');
    end
    

    mean_error = mean(abs(corr_norm-1))
    
    % Plot data
    len = size(sam_norm,2);
    figure;
    plot(0:1:len-1,sam_norm,'r');
    hold on
    plot(0:1:len-1,corr_norm,'g');
    legend(name + ' samples','Corrected ' + name + ' data',location="southeast")
    title("Sensor data norm for " + name + newline + "Mean absolute error after correction = " + mean_error)
    ylabel("||x||^2 norm of sensor data");
    xlabel("Sample number");
    ylim([0 max(sam_norm)*1.1])
end