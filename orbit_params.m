
%%Constant
altitude = 449e3;

earth.R = 6371e3;
moon.R = 1740e3;

earth.Mass = 5.972e24;
G = 6.674e-11;


semi_major = earth.R + altitude;
eccentricity = 0;
inclination = 97.21;
RAAN = 34.9;
periapsis_arg = 0;
true_anomaly = 45;

%time
year = 2026;
month = 05;
day = 21;
hour = 0;
minute = 0;
second = 0;

deltaAT = 37;



%%Satellite
NEO.mass = 230;
NEO.inertia = [30.5, 0, 0; 0, 20, 0; 0, 0, 40.2];

%initialise
q_icrf2b_initial = [-0.38; -0.3; 0.87; -0.06];
q_icrf2b_initial = q_icrf2b_initial/norm(q_icrf2b_initial);
omega_icrf2b_initial = [0.01;-0.01;0.01];


%% times
Ts = 0.1;
gyro.Ts = 0.1;
star.Ts = 1;
sun.Ts = 0.5;
mag.Ts = 1;
Kalman.Ts = 0.1;


%% gyro
% gyro.Ts = 0.5;
gyro.sigma_u = 4.6296e-06;  %% bias  both deg/s   using minimum reccomened value
gyro.sigma_v = 2.5e-3;  %%% this is standard deviation be careful  %% noise
gyro.seed  = 2134;
gyro.bias = [0.00,0.00,0.00]';

gyro2 = gyro;
gyro2.seed = 2135;

%%%%
gyro.fixed_bias = (1e-3) * 1; %%%% fixed bias radians
gyro.fixed_bias_vector = ((rand(3,1)) - 0.5) * 2;
gyro.fixed_bias_vector = (gyro.fixed_bias_vector)/norm(gyro.fixed_bias_vector) * gyro.fixed_bias ;

gyro2.fixed_bias_vector = ((rand(3,1)) - 0.5) * 2;
gyro2.fixed_bias_vector = (gyro2.fixed_bias_vector)/norm(gyro2.fixed_bias_vector) * gyro.fixed_bias ;




%%% scale factor 500ppm
%%% non linearing 20 ppm


%% star tracker

%ref frame https://www.mdpi.com/1424-8220/18/9/3106 % z along the boresign
star.boresight_b = [1;1;-1];  %% z

star.boresight_b = star.boresight_b/norm(star.boresight_b);

star.x = cross(star.boresight_b,[1;0;0])/norm(cross(star.boresight_b,[1;0;0]));
star.y = cross(star.boresight_b,star.x)/norm(cross(star.boresight_b,star.x));

star.dcm_b2star = [star.x';star.y';star.boresight_b'];

star.sun_avoidance_angle_deg = 35 * 0.01;
star.earth_avoidance_angle_deg = 22;


%%% random noise expected values semi optimistic. Dependant on slew : x2
%%% per deg/s of slew cross and x3 deg/s for about boresight
star.sigma_bore = 70/3 ;  %% 15  
star.sigma_cross = 11/3  ; %%% arcsec sigma ... 3

star.fixed_bias = (0.017 * 3600 * 0.1 + 3 * 1.5) * 0 ; %%%% fixed bias    such a hihgh fixed bisa
star.fixed_bias_vector = ((rand(3,1)) - 0.5) * 2;
star.fixed_bias_vector = (star.fixed_bias_vector)/norm(star.fixed_bias_vector);

star.max_slew = 1; %%%% 

% star.Ts = 1;
star.seed = randi([1,1000]);
star_Ts = star.Ts;

star.HF_sigma = [6.6; 6.6; 28]/3 /3600 * pi/180 * 0;
star.HF_def_tau = 0.0195; %%% def tau tau at 1 deg /sec total slew assume 1024 pixels so 0.0195 degree per pixel

star.LF_sigma = [9,9,51]/3 /3600 * pi/180 * 0;
star.LF_def_tau = 20;  %% assume 20 deg of fov default is at 1 deg/s


%%% star thermal <0.055 arcsec/°C from hydra multi head




%%%
% sun.Ts = 0.5;
sun.FOV_half_angle_degrees = 83;  %%% degrees assume square FOV
sun.sigma = deg2rad(0.1);  %% radians from small angle approximation satisfying the vector error
sun.axes = zeros(3,3,6);
sun.noise_unit = ones(3,1,6);
sun.noise_unit(3,:,:) = 0;
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

sun.bias = deg2rad(0.1) ; %% radians
for ii = 1:length(sun.axes)
    rand_vector = rand(3,1);
    rand_error = rand_vector/norm(rand_vector) * sun.bias * 1/2;
    sun.axes(:,:,ii) =  quat2dcm(  ([1;rand_error]/norm([1;rand_error]))'   ) * sun.axes(:,:,ii);
end
sun.dcm_sun2b = sun.axes;

sun.noise_seeds = randi([1,1000],3,1,6);
% sun.Ts = 2;
%% 70 deg/s max xslew should bve fine


%%%%magnetometer  assume nad ne
mag.x = [0;1;0];
mag.y = [0;0;1];
mag.z = [1;0;0];
mag.dcm_mag2b = [mag.x,mag.y,mag.z];


mag.fixed_misalign = 0.1; %%%% degrees
mag.fixed_misalign_vector = ((rand(3,1)) - 0.5) * 2;
mag.fixed_misalign_vector = (mag.fixed_misalign_vector)/norm(mag.fixed_misalign_vector);
mag.fixed_misalign_dcm = quat2dcm([cosd(mag.fixed_misalign);mag.fixed_misalign_vector * sind(mag.fixed_misalign) * 0.5]');
mag.bias = ((rand(3,1)) - 0.5) * 50 ;


% mag.Ts = 1;
mag.scale_fac = 0.01 ;
mag.scales = ((rand(3,3) - 0.5) * 2 .* [1,0.1,0.1;0.1,1,0.1;0.1,0.1,1] * mag.scale_fac + eye(3))^-1;   %% 1 percent
mag.sigma  = 120;
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



%%%%%    wahba
%%% 1 sun
b1 = [0.178;-0.9;-0.391];
b1 = b1/norm(b1);
sigma_1 = sun.sigma; %% rad

b2 = [4700;-2600;2.8e4];

sigma_2 = mag.sigma/norm(b2);
b2 = b2/norm(b2);

P_sigma_sigma = (sigma_2 ^2 * b1 * b1' + sigma_1^2 * b2 * b2') / norm(cross(b1,b2))^2 + sigma_1^2 * sigma_2^2 * cross(b1,b2) * cross(b1,b2)' / (sigma_1^2 + sigma_2^2) /norm(cross(b1,b2))^2;

sqrt((P_sigma_sigma(1,1))) * 180/pi * 3600;
sqrt((P_sigma_sigma(2,2))) * 180/pi * 3600;
sqrt((P_sigma_sigma(3,3))) * 180/pi * 3600;


a = 3;
Su = sigma_u_rad * (Kalman.Ts)^1.5 /sqrt((P_sigma_sigma(a,a)));
Sv = sigma_v_rad * (Kalman.Ts)^0.5 / sqrt((P_sigma_sigma(a,a)));
Se = 0;

gamma = sqrt(1 + Se^2 + 0.25 * Sv^2 + 1/48 * Su^2);
si = gamma + 0.25 * Su + 1/2 * sqrt( 2 * gamma * Su + Sv^2 + 1/3 * Su^2);
sigma_theta_rad_2 = sqrt((P_sigma_sigma(a,a))) * sqrt(1 - si^-2);
sigma_theta_arcsec_2 = sigma_theta_rad_2 * 180/pi * 3600


sigma_theta_rad_3 = sqrt((P_sigma_sigma(a,a))) * sqrt(-1 + si^2);
sigma_theta_arcsec_3 = sigma_theta_rad_3 * 180/pi * 3600


%%%% look at CW5000 of cubespace
wheel.rated_momentum_mNms = 500;
wheel.speed_at_rated_momentum_rads = 5200 * 2 * pi/60;
wheel.I_II = wheel.rated_momentum_mNms / 1000 / wheel.speed_at_rated_momentum_rads;

wheel.I = blkdiag(wheel.I_II /10,wheel.I_II /10, wheel.I_II );

a = sqrt(2/3);
d = a ;
b = sqrt(1 - a^2);
c = sqrt(1 - d^2);
wheel.axes = [a,-a, 0, 0;
              b, b, c, c;
              0, 0, d,-d];


%%% multiply by desired body moment or torque
wheel.distribution_matrix = 0.5 * [ 1/a, b/(b^2 + c^2),   0;
                                   -1/a, b/(b^2 + c^2),   0;
                                      0, c/(b^2 + c^2), 1/d;
                                      0, c/(b^2 + c^2),-1/d];
wheel.max_speed = 8000 * 2 * pi/60; %% rad/s
wheel.max_torque_Nm = 37e-3;

wheel.bias = 600; % rpm
wheel.omega_initial =  [wheel.bias,wheel.bias, wheel.bias, wheel.bias] * 2 * pi/60;
wheel.tau = 0.5; %% seconds

% desired h default
h_default = zeros(3,1);
for f = 1:4
 h_default = h_default + wheel.omega_initial(f) *  wheel.I_II * wheel.axes(:,f);

end

%% magnetor torquers  base on CR0150
mag_torque.axis_ideal = [1,0,0;
                         0,1,0
                         0,0,1];
mag_torque.axis = mag_torque.axis_ideal;
mag_torque.fixed_misalign = deg2rad(0.1);
for ii = 1:3
    misalign = rand(3,1);
    mag_torque.axis(:,ii) = mag_torque.axis(:,ii) + mag_torque.fixed_misalign * misalign/norm(misalign);
    mag_torque.axis(:,ii) = mag_torque.axis(:,ii)/norm(mag_torque.axis(:,ii));
end

mag_torque.max_dipole = 15;
mag_torque.gain = 63;
mag_torque.residual = 0.1 /100 * mag_torque.max_dipole;


mag_torque.tau = 40e-3 ; %% seconds


%% plotting approx model achieved from trial error
mag_torque.res = 19.9;
mag_torque.V_max = 5;
mag_torque.I_max = mag_torque.V_max/mag_torque.res;
% plot(linspace(0,5/19.9,1000),30 * tanh(63/28.6 * linspace(0,5/19.9 ,1000)))
mag_torque.tanh_max = 30;
mag_torque.tanh_gain = 63/28.6;
%% current is input
mag_torque.k = 0.005;



%% sim stuff
load('t_sim');
load('p_dyn');
load('cda');

nRep = 10;

t0 = t_sim(:);          % make sure it is column
T  = t0(end);           % duration of one block, if time starts at 0

t_rep = t0 + T*(0:nRep-1);
t_sim_long = t_rep(:);

p_dyn_long = repmat(p_dyn,nRep,1);

cda_long = repmat(cda,nRep,1);

%distance guess random
r_offset = [1;1;5] * 1e-2; %% cm to meter
%%assume drag in x

drag = p_dyn_long .* cda_long;

drag_simin = timeseries(drag, t_sim_long);

% aero_torque = cross(drag,r_offset);

tracking.kp = 0.5;
tracking.kd = 2;