
%%Constant
altitude = 600e3;

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
q_icrf2b_initial = [1; 0; 0; 0];
q_icrf2b_initial = q_icrf2b_initial/norm(q_icrf2b_initial);
omega_icrf2b_initial = [0.01;0.01;0.01];


%% times
Ts = 10;
gyro.Ts = 0.5;
star.Ts = 1;
sun.Ts = 1;
mag.Ts = 1;
Kalman.Ts = 0.5;


%% gyro
% gyro.Ts = 0.5;
gyro.sigma_u = 4.6296e-06;  %% bias  both deg/s
gyro.sigma_v = 1.6e-3;  %%% this is standard deviation be careful  %% noise
gyro.seed  = 2134;
gyro.bias = [0.00,0.00,0.00]';

gyro2 = gyro;
gyro2.seed = 2135;


%% star tracker

%ref frame https://www.mdpi.com/1424-8220/18/9/3106 % z along the boresign
star.boresight_b = [1;1;-1];  %% z

star.boresight_b = star.boresight_b/norm(star.boresight_b);

star.x = cross(star.boresight_b,[1;0;0])/norm(cross(star.boresight_b,[1;0;0]));
star.y = cross(star.boresight_b,star.x)/norm(cross(star.boresight_b,star.x));

star.dcm_b2star = [star.x';star.y';star.boresight_b'];

star.sun_avoidance_angle_deg = 35;
star.earth_avoidance_angle_deg = 22;


%%% random noise expected values semi optimistic. Dependant on slew : x2
%%% per deg/s of slew cross and x3 deg/s for about boresight
star.sigma_bore = 70/3 ;  %% 15  
star.sigma_cross = 11/3  ; %%% arcsec sigma ... 3

star.fixed_bias = (0.017 * 3600 + 5 * 1.5) * 0; %%%% fixed bias    such a hihgh fixed bisa
star.fixed_bias_vector = ((rand(3,1)) - 0.5) * 2;
star.fixed_bias_vector = (star.fixed_bias_vector)/norm(star.fixed_bias_vector);

star.max_slew = 1; %%%% 

% star.Ts = 1;
star.seed = 2133;
star_Ts = star.Ts;

star.HF_sigma = [6.6; 6.6; 28]/3 /3600 * pi/180 * 0;
star.HF_def_tau = 0.01; %%% def tau tau at 1 deg /sec total slew

star.LF_sigma = [9,9,51]/3 /3600 * pi/180 * 0;
star.LF_def_tau = 20;


%%% star thermal <0.055 arcsec/°C from hydra multi head




%%%
% sun.Ts = 0.5;
sun.FOV_half_angle_degrees = 60;  %%% degrees assume square FOV
sun.sigma = deg2rad(0.5);  %% radians from small angle approximation satisfying the vector error
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
sun.dcm_sun2b_ideal = sun.axes;

sun.bias = deg2rad(0.1); %% radians
for ii = 1:length(sun.axes)
    rand_vector = rand(3,1);
    rand_error = rand_vector/norm(rand_vector) * sun.bias * 1/2;
    sun.axes(:,:,ii) =  quat2dcm(  ([1;rand_error]/norm([1;rand_error]))'   ) * sun.axes(:,:,ii);
end
sun.dcm_sun2b = sun.axes;

sun.noise_seeds = randi([1,1000],3,1,6);
% sun.Ts = 2;


%%%%magnetometer
mag.x = [0;1;0];
mag.y = [0;0;1];
mag.z = [1;0;0];
mag.dcm_mag2b = [mag.x,mag.y,mag.z];


mag.fixed_misalign = 0.1; %%%% degrees
mag.fixed_misalign_vector = ((rand(3,1)) - 0.5) * 2;
mag.fixed_misalign_vector = (mag.fixed_misalign_vector)/norm(mag.fixed_misalign_vector);
mag.fixed_misalign_dcm = quat2dcm([cosd(mag.fixed_misalign);mag.fixed_misalign_vector * sind(mag.fixed_misalign) * 0.5]');
mag.bias = ((rand(3,1)) - 0.5) * 50;


% mag.Ts = 1;
mag.scale_fac = 0.01;
mag.scales = ((rand(3,3) - 0.5) * 2 .* [1,0.1,0.1;0.1,1,0.1;0.1,0.1,1] * mag.scale_fac + eye(3))^-1;   %% 1 percent
mag.sigma  = 40;
mag.co_var = ones(3,1) * mag.sigma^2;
mag.noise_seeds = randi([1,1000],3,1);

mag2 = mag;
mag2.scales = ((rand(3,3) - 0.5) * 2 .* [1,0.1,0.1;0.1,1,0.1;0.1,0.1,1] * mag.scale_fac + eye(3))^-1;   %% 1 percent
mag2.co_var = ones(3,1) * mag2.sigma^2;
mag2.noise_seeds = randi([1,1000],3,1);

mag.R_k = eye(3) * mag.sigma^2;

%%EKF
% q_initial  = quatmultiply( q_icrf2b_initial', [cosd(20),sind(20)*([1,2,5]/norm([1,2,5]))])';
q_initial = q_icrf2b_initial;
x_initial = zeros(6,1); 

P_initial = blkdiag(eye(3) * gyro.sigma_v^2 * 1e4 ,eye(3) * gyro.sigma_u^2 * 1e5);  %% initial state estiatme covar

Q_c = blkdiag(eye(3) * gyro.sigma_v^2/2 ,eye(3) * gyro.sigma_u^2/2 );  %% process noise covar not used anymore

R_k_star = blkdiag(star.sigma_cross^2,star.sigma_cross^2,star.sigma_bore^2) * ((1/3600) * pi/180)^2;   %%% meas noise in the start trcker frame

% R_k_sun_single = eye(3) * sun.sigma^2;

sigma_st = sqrt( (star.sigma_bore^2 + star.sigma_cross ^2 * 2)/3 );
sigma_st_rad = sigma_st/3600 * pi/180;       % arcsec → rad
sigma_v_rad  = gyro.sigma_v * pi/180 /sqrt(2);         % deg/√s → rad/√s
sigma_u_rad  = gyro.sigma_u * pi/180 /sqrt(2);         % deg/s/√s → rad/s/√s
% 
variance = sigma_st_rad * sigma_v_rad * sqrt(Kalman.Ts) * ...
           sqrt(1 + (1e-9 * sigma_u_rad * sigma_st_rad / sigma_v_rad^3) * sqrt(Kalman.Ts));

sigma_theta_rad    = sqrt(variance);
sigma_theta_arcsec = sigma_theta_rad * 180/pi * 3600



%%% other
Su = sigma_u_rad * (Kalman.Ts)^1.5 /sigma_st_rad;
Sv = sigma_v_rad * (Kalman.Ts)^0.5 / sigma_st_rad;
Se = 0;

gamma = sqrt(1 + Se^2 + 0.25 * Sv^2 + 1/48 * Su^2);
si = gamma + 0.25 * Su + 1/2 * sqrt( 2 * gamma * Su + Sv^2 + 1/3 * Su^2);
sigma_theta_rad_2 = sigma_st_rad * sqrt(1 - si^-2);
sigma_theta_arcsec_2 = sigma_theta_rad_2 * 180/pi * 3600


sigma_theta_rad_3 = sigma_st_rad * sqrt(-1 + si^2);
sigma_theta_arcsec_3 = sigma_theta_rad_3 * 180/pi * 3600