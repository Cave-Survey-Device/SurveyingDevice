% Configure sources
function plot_3d_data( FILE_samples, FILE_corrections, name)

    % Import data
    samples =      importdata(FILE_samples);
    corrections =  importdata(FILE_corrections);
    
    % Calculate normal vector of data
    if size(samples,1) ~= 3
        samples = samples';
        corrections = corrections';
    end

    x = samples(1,:)';
    y = samples(2,:)';
    z = samples(3,:)';

    %draw data
    figure,
    scatter3( x, y, z,'filled');
    hold on;
    [ center, radii, evecs, v, chi2 ] = ellipsoid_fit( [ x y z ], '' );

    %draw fit
    mind = [-1.5 -1.5 -1.5]
    maxd = [1.5 1.5 1.5]
    nsteps = 50;
    step = ( maxd - mind ) / nsteps;
    [ x, y, z ] = meshgrid( linspace( mind(1) - step(1), maxd(1) + step(1), nsteps ), linspace( mind(2) - step(2), maxd(2) + step(2), nsteps ), linspace( mind(3) - step(3), maxd(3) + step(3), nsteps ) );
    Ellipsoid = v(1) *x.*x +   v(2) * y.*y + v(3) * z.*z + ...
              2*v(4) *x.*y + 2*v(5)*x.*z + 2*v(6) * y.*z + ...
              2*v(7) *x    + 2*v(8)*y    + 2*v(9) * z;
    p = patch( isosurface( x, y, z, Ellipsoid, -v(10) ) );
    hold on;
    [X,Y,Z] = sphere;
    s = surf(X,Y,Z);
    s.FaceAlpha = 0.5;
    s.FaceColor = 'green';
    s.EdgeColor = 'none';

    hold off;
    set( p, 'FaceColor', 'red', 'EdgeColor', 'none', 'FaceAlpha', 0.75 );
    axis vis3d equal;

    legend(name + ' samples','Best fit ellipsoid for ' + name + ' samples', 'Ellipsoid for ideal sensor',location="southeast")
    title("Sensor data for " + name + newline)


end