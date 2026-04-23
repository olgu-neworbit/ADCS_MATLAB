
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

Ts = 1;



%% gyro
gyro.Ts = 0.5;
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

star.fixed_bias = 5; %%%% fixed bias
star.fixed_bias_vector = ((rand(3,1)) - 0.5) * 2;
star.fixed_bias_vector = (star.fixed_bias_vector)/norm(star.fixed_bias_vector);

star.max_slew = 10; %%%% 

star.Ts = 1;
star.seed = 2133;
star_Ts = star.Ts;


%%%
sun.Ts = 0.5;
sun.FOV_half_angle_degrees = 60;  %%% degrees assume square FOV
sun.sigma = 0.00001;  %% degrees??
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
sun.Ts = 2;


%%%%magnetometer
mag.x = [0;1;0];
mag.y = [0;0;1];
mag.z = [1;0;0];
mag.dcm_mag2b = [mag.x,mag.y,mag.z];


mag.fixed_misalign = 1 * 0; %%%% degrees
mag.fixed_misalign_vector = ((rand(3,1)) - 0.5) * 2;
mag.fixed_misalign_vector = (mag.fixed_misalign_vector)/norm(mag.fixed_misalign_vector);
mag.fixed_misalign_dcm = quat2dcm([cosd(1);mag.fixed_misalign_vector * sind(mag.fixed_misalign)]');
mag.bias = ((rand(3,1)) - 0.5) * 500 * 0;


mag.Ts = 1;

mag.scales = (0.0001*(rand(3,3) - 0.5) /50 .* [1,0.1,0.1;0.1,1,0.1;0.1,0.1,1] + eye(3))^-1;   %% 1 percent
mag.sigma  = 100 * 0.001;
mag.co_var = ones(3,1) * mag.sigma^2;
mag.noise_seeds = randi([1,1000],3,1);

mag2 = mag;
mag2.scales = (0.0001*(rand(3,3) - 0.5) /50 .* [1,0.1,0.1;0.1,1,0.1;0.1,0.1,1] + eye(3))^-1;   %% 1 percent
mag2.sigma  = 100 * 0.0001;
mag2.co_var = ones(3,1) * mag2.sigma^2;
mag2.noise_seeds = randi([1,1000],3,1);

mag.R_k = eye(3) * mag.sigma^2;

%%EKF
q_initial  = quatmultiply( q_icrf2b_initial', [cosd(20),sind(20)*([1,2,5]/norm([1,2,5]))])';
x_initial = zeros(6,1); 
Kalman.Ts = gyro.Ts;

P_initial = blkdiag(eye(3) * gyro.sigma_v^2 ,eye(3) * gyro.sigma_u^2) * 1e-6;  %% initial state estiatme covar

Q_c = blkdiag(eye(3) * gyro.sigma_v^2/2 ,eye(3) * gyro.sigma_u^2/2);  %% process noise covar

R_k_star = blkdiag(star.sigma_cross^2,star.sigma_cross^2,star.sigma_bore^2) * (1/3600) * pi/180;   %%% meas noise in the start trcker frame

% R_k_sun_single = eye(3) * sun.sigma^2;

% 
% 
% if mag_valid
%     %%%%%%%%%% prev  mag_XYZ_icrf_model,mag_XYZ_body_meas,mag_valid
%     P_k_k1 = P_k_k;
%     x_k_k1 = x_k_k;
%     q_k_k1 = q_k_k;
% 
%     %% normalise
%     mag_XYZ_icrf_model = mag_XYZ_icrf_model/norm(mag_XYZ_icrf_model);
% 
%     mag_XYZ_body_meas = mag_XYZ_body_meas/norm(mag_XYZ_body_meas);
% 
% 
% 
% 
% 
%     %% get current rotation matrix
%     q_k_k1;
%     dcm_icrf2b = quat2dcm(q_k_k1');
% 
% 
%     %%%kalman gain and mearuement error    
%     R_k_mag = mag.R_k;
% 
% 
%     mag_XYZ_body_model = dcm_icrf2b * mag_XYZ_icrf_model;
% 
%     % R_k_sun_one = 10000000 *sun.sigma^2 * (eye(3) - sat2sun_body*sat2sun_body') + 10000000 * sun.sigma^2 * (sat2sun_body*sat2sun_body');
%     % R_k_sun = kron(eye(FSS_no), R_k_sun_one);
% 
%     mag_XYZ_body_model_skew = [0, -mag_XYZ_body_model(3), mag_XYZ_body_model(2);
%         mag_XYZ_body_model(3), 0, -mag_XYZ_body_model(1);
%         -mag_XYZ_body_model(2), mag_XYZ_body_model(1), 0];
% 
%     H_k_row = [mag_XYZ_body_model_skew,zeros(3,3)];
%     H_k = repmat(H_k_row,FSS_no,1);
% 
% 
%     K_k = P_k_k1*H_k'/(H_k*P_k_k1*H_k' + R_k_mag);
% 
%     % sun_vectors_valid_body = pagemtimes(dcm_icrf2b,sun_vectors_valid);
%     y_k = mag_XYZ_body_meas;
% 
%     h_x_k = mag_XYZ_body_model;
% 
% 
%     x_k_k = x_k_k1 + K_k * (y_k - h_x_k);
%     % 
%     P_k_k = (eye(6,6) - K_k * H_k) * P_k_k1 * (eye(6,6) - K_k * H_k)' + K_k * R_k_mag * K_k';
% 
%     %% get quat
%     dq = [1;0.5 * x_k_k(1:3)];
%     dq = dq/norm(dq);
% 
%     q_k_k = quatmultiply(q_k_k1',dq')';
%     q_k_k = q_k_k/norm(q_k_k);
% 
%     x_k_k(1:3) = 0;
% else
%     % x_k_k = x_k_k;
%     % P_k_k = P_k_k;
% end
% 
% 
% if any(sun_valid)
%     %%%%%%%%%% Star
%     P_k_k1 = P_k_k;
%     x_k_k1 = x_k_k;
% 
%     %%%valid FSS no
%     FSS_no = sum(sun_valid == 1);
%     sun_vectors_valid = sun_vectors(:,:,sun_valid);
% 
%     %% normalise
%     sat2sun_icrf = sat2sun_icrf/norm(sat2sun_icrf);
% 
%     for kk = 1:FSS_no
%         sun_vectors_valid(:,:,kk) = sun_vectors_valid(:,:,kk)/norm(sun_vectors_valid(:,:,kk));
%     end
% 
% 
%     %% measurment matrix simple
% 
% 
% 
%     % H_k = [eye(3,3),zeros(3,3)];
% 
% 
%     %% get current rotation matrix
%     q_k_k;
%     dcm_icrf2b = quat2dcm(q_k_k');
% 
% 
%     %%%kalman gain and mearuement error    
%     R_k_sun = eye(FSS_no*3) * sun.sigma^2;
% 
%     sat2sun_body = dcm_icrf2b * sat2sun_icrf;
%     sat2sun_body_skew = [0, -sat2sun_body(3), sat2sun_body(2);
%         sat2sun_body(3), 0, -sat2sun_body(1);
%         -sat2sun_body(2), sat2sun_body(1), 0];
% 
%     H_k_row = [sat2sun_body_skew,zeros(3,3)];
%     H_k = repmat(H_k_row,FSS_no,1);
% 
% 
%     K_k = P_k_k1*H_k'/(H_k*P_k_k1*H_k' + R_k_sun);
% 
%     % sun_vectors_valid_body = pagemtimes(dcm_icrf2b,sun_vectors_valid);
%     y_k = reshape(sun_vectors_valid, [], 1);%%% measurment in body frame stacked already in body pay attention
% 
%     h_x_k = repmat(sat2sun_body,FSS_no,1);
% 
% 
%     x_k_k = x_k_k1 + K_k * (y_k - h_x_k);
%     % 
%     P_k_k = (eye(6,6) - K_k * H_k) * P_k_k1 * (eye(6,6) - K_k * H_k)' + K_k * R_k_sun * K_k';
% 
%     %% get quat
%     dq = [1;0.5 * x_k_k(1:3)];
%     dq = dq/norm(dq);
% 
%     q_k_k = quatmultiply(q_k_k1',dq')';
%     q_k_k = q_k_k/norm(q_k_k);
% 
%     x_k_k(1:3) = 0;
% else
%     % x_k_k = x_k_k;
%     % P_k_k = P_k_k;
% end