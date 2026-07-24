clear
close all
sun_data = 'JPL_planet_motion_tables/sun_results.txt';
mercury_data = 'JPL_planet_motion_tables/mercury_results.txt';
venus_data = 'JPL_planet_motion_tables/venus_results.txt';
earth_data = 'JPL_planet_motion_tables/earth_results.txt';
mars_data = 'JPL_planet_motion_tables/mars_results.txt';
jupiter_data = 'JPL_planet_motion_tables/jupiter_results.txt';
saturn_data = 'JPL_planet_motion_tables/saturn_results.txt';

uranus_data = 'JPL_planet_motion_tables/uranus_results.txt';
neptune_data = 'JPL_planet_motion_tables/neptune_results.txt';
pluto_data = 'JPL_planet_motion_tables/pluto_results.txt';
% Read the file into a MATLAB 
fprintf('reading sun\n');     sunData = read_horizons_ephemeris(sun_data);
fprintf('reading mercury\n'); mercuryData = read_horizons_ephemeris(mercury_data);
fprintf('reading venus\n');   venusData = read_horizons_ephemeris(venus_data);
fprintf('reading earth\n');   earthData = read_horizons_ephemeris(earth_data);
fprintf('reading mars\n');    marsData = read_horizons_ephemeris(mars_data);
fprintf('reading jupiter\n'); jupiterData = read_horizons_ephemeris(jupiter_data);
fprintf('reading saturn\n');  saturnData= read_horizons_ephemeris(saturn_data);
fprintf('reading uranus\n');  uranusData= read_horizons_ephemeris(uranus_data);
fprintf('reading neptune\n'); neptuneData= read_horizons_ephemeris(neptune_data);
fprintf('reading pluto\n');   plutoData= read_horizons_ephemeris(pluto_data);
% Display the first few rows of data
%head(marsData)

% Access a specific column directly
pluto_limit = 7.4e9;

plot3(sunData.X, sunData.Y,sunData.Z);
hold on
plot3(mercuryData.X, mercuryData.Y,mercuryData.Z);
plot3(venusData.X, venusData.Y,venusData.Z);
plot3(earthData.X, earthData.Y,earthData.Z);
plot3(marsData.X, marsData.Y,marsData.Z);
plot3(jupiterData.X, jupiterData.Y,jupiterData.Z);
plot3(saturnData.X, saturnData.Y,saturnData.Z);
plot3(uranusData.X, uranusData.Y,uranusData.Z);
plot3(neptuneData.X, neptuneData.Y,neptuneData.Z);
%plot3(plutoData.X, plutoData.Y,plutoData.Z);
% Set identical bounds on all three spatial axes
xlim([-pluto_limit, pluto_limit]);
ylim([-pluto_limit, pluto_limit]);
zlim([-pluto_limit, pluto_limit]);
plot3(plutoData.X, plutoData.Y,plutoData.Z);
%axis equal;
axis vis3d
legend('sun', 'mercury', 'venus', 'earth', 'mars', 'jupiter', 'saturn', 'neptune', 'pluto');
grid on



%title('Mars Orbit Path');


function data = read_horizons_ephemeris(filename)
    % READ_HORIZONS_EPHEMERIS Parses NASA Horizons cartesian ephemeris output
    %
    % Output: 
    %   data - A table containing columns for JDTDB, CalendarTime, X, Y, Z, 
    %          VX, VY, VZ, LT, RG, and RR.

    % Open the file for reading
    fid = fopen(filename, 'r');
    if fid == -1
        error('Could not open file: %s', filename);
    end
    
    % Clean up and close the file automatically if an error occurs
    cleanup = onCleanup(@() fclose(fid));
    
    % Step 1: Find the $$SOE marker
    found_soe = false;
    while ~feof(fid)
        line = strtrim(fgetl(fid));
        if strcmp(line, '$$SOE')
            found_soe = true;
            break;
        end
    end
    
    if ~found_soe
        error('Could not find the starting marker $$SOE in the file.');
    end
    
    % Preallocate cell arrays to hold the parsed values for speed
    jd_list   = [];
    time_list = {};
    x_list    = []; y_list  = []; z_list  = [];
    vx_list   = []; vy_list = []; vz_list = [];
    lt_list   = []; rg_list = []; rr_list = [];
    
    % Step 2: Read line-by-line until $$EOE is reached
    while ~feof(fid)
        line1 = strtrim(fgetl(fid));
        
        % Check for the end of the data section
        if strcmp(line1, '$$EOE') || isempty(line1)
            if strcmp(line1, '$$EOE')
                break;
            end
            continue; % Skip blank lines
        end
        
        % Read the next 3 lines containing the vector components
        line2 = strtrim(fgetl(fid));
        line3 = strtrim(fgetl(fid));
        line4 = strtrim(fgetl(fid));
        
        % Parse Line 1: "2440759.500000000 = A.D. 1970-Jun-22 00:00:00.0000 TDB"
        tokens = split(line1, '=');
        if numel(tokens) < 2
            continue; % Edge case/malformed line block
        end
        jd_val   = sscanf(tokens{1}, '%f');
        time_val = strtrim(tokens{2});
        
        % Parse Line 2: "X =... Y =... Z =..."
        xyz = sscanf(line2, 'X =%f Y =%f Z =%f');
        
        % Parse Line 3: "VX=... VY=... VZ=..."
        vxvyvz = sscanf(line3, 'VX=%f VY=%f VZ=%f');
        
        % Parse Line 4: "LT=... RG=... RR=..."
        ltrgrr = sscanf(line4, 'LT=%f RG=%f RR=%f');
        
        % Store the parsed values into arrays
        jd_list(end+1, 1)   = jd_val;
        time_list{end+1, 1} = time_val;
        
        x_list(end+1, 1) = xyz(1);
        y_list(end+1, 1) = xyz(2);
        z_list(end+1, 1) = xyz(3);
        
        vx_list(end+1, 1) = vxvyvz(1);
        vy_list(end+1, 1) = vxvyvz(2);
        vz_list(end+1, 1) = vxvyvz(3);
        
        lt_list(end+1, 1) = ltrgrr(1);
        rg_list(end+1, 1) = ltrgrr(2);
        rr_list(end+1, 1) = ltrgrr(3);
    end
    
    % Step 3: Bundle everything together into a clean MATLAB Table
    data = table(jd_list, time_list, x_list, y_list, z_list, ...
                 vx_list, vy_list, vz_list, lt_list, rg_list, rr_list, ...
                 'VariableNames', {'JDTDB', 'CalendarTime', 'X', 'Y', 'Z', ...
                                   'VX', 'VY', 'VZ', 'LT', 'RG', 'RR'});
end
