
%%Constant
altitude = 150e3;

earth.R = 6371e3;

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
omega_icrf2b_initial = [1;-1;0];

Ts = 10;



%% gyro
gyro.Ts = 0.5;
gyro.sigma_u = 1e-3;
gyro.sigma_v = 1e-3;  %%% this is standard deviation be careful


%% star tracker

%ref frame https://www.mdpi.com/1424-8220/18/9/3106 % z along the boresign
star.boresight_b = [0;1;0];  %% z

star.boresight_b = star.boresight_b/norm(star.boresight_b);

star.x = cross(star.boresight_b,[1;0.1;0])/norm(cross(star.boresight_b,[1;0.1;0]));
star.y = cross(star.boresight_b,star.x)/norm(cross(star.boresight_b,star.x));

star.dcm_b2star = [star.x';star.y';star.boresight_b'];

star.sun_avoidance_angle_deg = 20;
star.earth_avoidance_angle_deg = 10;
