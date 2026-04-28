
%%Constant
altitude = 250e3;

earth.R = 6371e3;
moon.R = 1740e3;

semi_major = earth.R + altitude;
eccentricity = 0;
inclination = 170;
RAAN = 0;
periapsis_arg = 0;
true_anomaly = 10;

%time
year = 2020;
month = 1;
day = 1;
hour = 12;
minute = 0;
second = 0;

deltaAT = 37;



%%Satellite
NEO.mass = 230;
NEO.inertia = [30.5, 0, 0; 0, 20, 0; 0, 0, 40.2];

%initialise
q_icrf2b_initial = [0; -0.38; 0.92; 0];
q_icrf2b_initial = q_icrf2b_initial/norm(q_icrf2b_initial);
omega_icrf2b_initial = [0;0;0];


%% times
Ts = 10;
gyro.Ts = 0.5;
star.Ts = 1;
sun.Ts = 1;
mag.Ts = 1;
Kalman.Ts = gyro.Ts;


%% gyro
% gyro.Ts = 0.5;
gyro.sigma_u = 3e-5;  %% bias
gyro.sigma_v = 1e-4;  %%% this is standard deviation be careful  %% noise
gyro.seed  = 2134;

gyro2 = gyro;
gyro2.seed = 2135;


%% star tracker

%ref frame https://www.mdpi.com/1424-8220/18/9/3106 % z along the boresign
star.boresight_b = [0;1;1];  %% z

star.boresight_b = star.boresight_b/norm(star.boresight_b);

star.x = cross(star.boresight_b,[1;0;0])/norm(cross(star.boresight_b,[1;0;0]));
star.y = cross(star.boresight_b,star.x)/norm(cross(star.boresight_b,star.x));

star.dcm_b2star = [star.x';star.y';star.boresight_b'];

star.sun_avoidance_angle_deg = 20;
star.earth_avoidance_angle_deg = 10;

star.sigma_bore = 25;  %% 25
star.sigma_cross = 3; %%% arcsec sigma  3

star.fixed_bias = 1; %%%% fixed bias
star.fixed_bias_vector = ((rand(3,1)) - 0.5) * 2;
star.fixed_bias_vector = (star.fixed_bias_vector)/norm(star.fixed_bias_vector);

star.max_slew = 10; %%%% 

% star.Ts = 1;
star.seed = 2133;
star_Ts = star.Ts;


%%%
% sun.Ts = 0.5;
sun.FOV_half_angle_degrees = 60;  %%% degrees assume square FOV
sun.sigma = 0.01;  %% radians from small angle approximation satisfying the vector error
sun.axes = zeros(3,3,6);
sun.axes(:,:,1) = [1,0,0;
                   0,1,0;
                   0,0,1];
sun.axes(:,:,2) = [1,0,0;
                   0,1,0;
                   0,0,-1];
sun.axes(:,:,3) = [1,0,0;
                   0,0,1;
                   0,1,0];
sun.axes(:,:,4) = [1,0,0;
                   0,0,-1;
                   0,1,0];
sun.axes(:,:,5) = [0,0,1;
                   0,1,0;
                   1,0,0];
sun.axes(:,:,6) = [0,0,-1;
                   0,1,0;
                   1,0,0];
sun.dcm_sun2b = sun.axes;
sun.noise_seeds = randi([1,1000],3,1,6);
% sun.Ts = 2;


%%%%magnetometer
mag.x = [0;1;0];
mag.y = [0;0;1];
mag.z = [1;0;0];
mag.dcm_mag2b = [mag.x,mag.y,mag.z];


mag.fixed_misalign = 0.01; %%%% degrees
mag.fixed_misalign_vector = ((rand(3,1)) - 0.5) * 2;
mag.fixed_misalign_vector = (mag.fixed_misalign_vector)/norm(mag.fixed_misalign_vector);
mag.fixed_misalign_dcm = quat2dcm([cosd(mag.fixed_misalign);mag.fixed_misalign_vector * sind(mag.fixed_misalign)]');
mag.bias = ((rand(3,1)) - 0.5) * 10;


% mag.Ts = 1;

mag.scales = ((rand(3,3) - 0.5) /100 .* [1,0.1,0.1;0.1,1,0.1;0.1,0.1,1] + eye(3))^-1;   %% 1 percent
mag.sigma  = 100;
mag.co_var = ones(3,1) * mag.sigma^2;
mag.noise_seeds = randi([1,1000],3,1);

mag2 = mag;
mag2.scales = ((rand(3,3) - 0.5) /100 .* [1,0.1,0.1;0.1,1,0.1;0.1,0.1,1] + eye(3))^-1;   %% 1 percent
mag2.sigma  = 100;
mag2.co_var = ones(3,1) * mag2.sigma^2;
mag2.noise_seeds = randi([1,1000],3,1);

mag.R_k = eye(3) * mag.sigma^2;

%%EKF
% q_initial  = quatmultiply( q_icrf2b_initial', [cosd(20),sind(20)*([1,2,5]/norm([1,2,5]))])';
q_initial = q_icrf2b_initial;
x_initial = zeros(6,1); 
Kalman.Ts = gyro.Ts;

P_initial = blkdiag(eye(3) * gyro.sigma_v^2 ,eye(3) * gyro.sigma_u^2) * 1e-6;  %% initial state estiatme covar

% Q_c = blkdiag(eye(3) * gyro.sigma_v^2/2 ,eye(3) * gyro.sigma_u^2/2 );  %% process noise covar

R_k_star = blkdiag(star.sigma_cross^2,star.sigma_cross^2,star.sigma_bore^2) * ((1/3600) * pi/180)^2;   %%% meas noise in the start trcker frame

% R_k_sun_single = eye(3) * sun.sigma^2;
