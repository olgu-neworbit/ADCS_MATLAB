function [q_k_k,x_k_k] = update_MEKF(omega_icrf2b_gyro,q_icrf2b_star,star_valid,sun_vectors,sun_valid, mag_XYZ_icrf_model,mag_XYZ_body_meas,mag_valid, sat2sun_icrf,x_initial, q_initial, Kalman, P_initial, Q_c, star, R_k_star, sun, mag, gyro)


persistent q_k_k_state P_k_k_state x_k_k_state tick

if isempty(q_k_k_state) 
    x_k_k_state = x_initial;   %%% error state
    q_k_k_state = q_initial; %%%%% initial quaternion
    P_k_k_state = P_initial;
    tick = uint64(0);
end
x_k1_k1 = x_k_k_state;
q_k1_k1 = q_k_k_state;
P_k1_k1 = P_k_k_state;


%%%%%%%%
% x = [a;bias]
omega_icrf2b_gyro_unbiased = omega_icrf2b_gyro - x_k1_k1(4:6);


%%%%% Omega matrix
w_cross = [  0   -omega_icrf2b_gyro_unbiased(3)   omega_icrf2b_gyro_unbiased(2);
            omega_icrf2b_gyro_unbiased(3)    0   -omega_icrf2b_gyro_unbiased(1);
           -omega_icrf2b_gyro_unbiased(2)   omega_icrf2b_gyro_unbiased(1)    0 ];
w_mat = [0        -omega_icrf2b_gyro_unbiased';
         omega_icrf2b_gyro_unbiased    -w_cross];




%%% Get a priori quaternion
% q_icrf2b_dot = 0.5 * w_mat * q_k1_k1;
% x_k_k1 = [zeros(3,1);x_k1_k1(4:6)];
% 
% q_k_k1 = q_k1_k1 + q_icrf2b_dot * Kalman.Ts;

%% NEW FROM BOOK
d_theta = 0.5 * norm(omega_icrf2b_gyro_unbiased) * Kalman.Ts;
PHI_k = sin(d_theta) * omega_icrf2b_gyro_unbiased/norm(omega_icrf2b_gyro_unbiased);

OMEGA_k = [cos(d_theta) * eye(3) - skew_sym_mat(PHI_k), PHI_k;
    -PHI_k', cos(d_theta)];

q_k_k1_convention = OMEGA_k * [q_k1_k1(2:4);q_k1_k1(1)];
q_k_k1 = [q_k_k1_convention(4);q_k_k1_convention(1:3)];
x_k_k1 = [zeros(3,1);x_k1_k1(4:6);x_k1_k1(7:9)];




q_k_k1 = q_k_k1 / norm(q_k_k1);
    %%%get continious model
    
    % F = [-w_cross,  -eye(3,3)  ; ...
    %     zeros(3,3),  zeros(3,3)];   %%%% skew symetric w cross
    
    G = [-eye(3,3), zeros(3,3),zeros(3,3); ...
        zeros(3,3), eye(3,3),zeros(3,3);
        zeros(3,3),zeros(3,3),eye(3,3)];
    
    
    
    %%%
    %%old old
    % % % % Q_c = [eye(9,9)] * 1e-6;
    % % % % Q_k = G * Q_c * G' * Kalman.Ts;
    % % % % Q_k = Q_c;
    % old
%     Phi_k = eye(9,9) + F * Kalman.Ts;
%     % Q_k = [gyro_sigma_rad_v^2*Kalman.Ts/2 + 1/3 * gyro_sigma_rad_u^2/2 * Kalman.Ts * eye(3),-(0.5 * gyro_sigma_rad_u^2/2 * Kalman.Ts^2) * eye(3);-(0.5*gyro_sigma_rad_u^2/2 * Kalman.Ts^2) * eye(3),(gyro_sigma_rad_u^2/2 * Kalman.Ts  * eye(3) )];
% Q_k = [(gyro_sigma_rad_v^2*Kalman.Ts/2 + 1/3 * gyro_sigma_rad_u^2/2 * Kalman.Ts^3) * eye(3),-(0.5 * gyro_sigma_rad_u^2/2 * Kalman.Ts^2) * eye(3); ...
%     -(0.5*gyro_sigma_rad_u^2/2 * Kalman.Ts^2) * eye(3),(gyro_sigma_rad_u^2/2 * Kalman.Ts  * eye(3) )];
%     P_k_k1 = Phi_k * P_k1_k1 * Phi_k' + Q_k;
%% NEW FROM BOOK
gyro_sigma_rad_u = gyro.sigma_u * pi/180;
gyro_sigma_rad_v = gyro.sigma_v * pi/180;

Q_k = [(gyro_sigma_rad_v^2*Kalman.Ts/2 + 1/3 * gyro_sigma_rad_u^2/2 * Kalman.Ts^3) * eye(3),-(0.5 * gyro_sigma_rad_u^2/2 * Kalman.Ts^2) * eye(3),zeros(3,3); ...
    -(0.5*gyro_sigma_rad_u^2/2 * Kalman.Ts^2) * eye(3),(gyro_sigma_rad_u^2/2 * Kalman.Ts  * eye(3) ),zeros(3,3);
    zeros(3,3),zeros(3,3),eye(3) * 1e-10];

Phi_k_11 = eye(3) - w_cross * sin(norm(omega_icrf2b_gyro_unbiased * Kalman.Ts))/norm(omega_icrf2b_gyro_unbiased) ...
    + w_cross * w_cross * (1 - cos(norm(omega_icrf2b_gyro_unbiased) * Kalman.Ts))/norm(omega_icrf2b_gyro_unbiased)^2;
Phi_k_12 = w_cross * (1 - cos(norm(omega_icrf2b_gyro_unbiased) * Kalman.Ts))/norm(omega_icrf2b_gyro_unbiased)^2 - eye(3) * Kalman.Ts ...
    -w_cross * w_cross * (norm(omega_icrf2b_gyro_unbiased) * Kalman.Ts - sin(norm(omega_icrf2b_gyro_unbiased) * Kalman.Ts))/norm(omega_icrf2b_gyro_unbiased)^3;

Phi_k = [Phi_k_11,Phi_k_12,zeros(3,3);
    zeros(3,3),eye(3,3),zeros(3,3);
    zeros(3,3),zeros(3,3),eye(3,3)];
P_k_k1 = Phi_k * P_k1_k1 * Phi_k' + G * Q_k * G';

if star_valid && mod(tick,uint64(star.Ts/Kalman.Ts)) == uint64(0)
    %%%%%%%%%% Star

    
    
    %% measurment matrix simple   % star tracket
    H_k = [eye(3,3),zeros(3,3),-eye(3,3) * star.dcm_b2star' * 0];
    
    
    %%%kalman gain and mearuement error
    % R_k = eye(3,3) * 1e-6;
    
    R_k = star.dcm_b2star' * R_k_star * star.dcm_b2star;
    K_k = P_k_k1*H_k'/(H_k*P_k_k1*H_k' + R_k);
    
    
    dq_star = quatmultiply(quatinv(q_k_k1'),q_icrf2b_star')';
    if dq_star(1) < 0
        dq_star = -dq_star;
    end
    x_star = 2*[dq_star(2:4)];
    
    x_k_k = x_k_k1 + K_k * (x_star - star.dcm_b2star' * x_k_k1(7:9) - H_k * x_k_k1);
    % P_k_k = (eye(9,9) - K_k * H_k)*P_k_k1;    %%% changet to Joseph form
    P_k_k = (eye(9,9) - K_k * H_k) * P_k_k1 * (eye(9,9) - K_k * H_k)' + K_k * R_k * K_k';


    %% get quat
    dq = [1;0.5 * x_k_k(1:3)];
    dq = dq/norm(dq);
    
    q_k_k = quatmultiply(q_k_k1',dq')';
    q_k_k = q_k_k/norm(q_k_k);
    
    x_k_k(1:3) = 0;
    

    %%Covar reset if wanted from file:///C:/Users/OlguPilav/Downloads/Covariance_Reset_JGCD_2023.pdf
    % qhat_13 = dq(2:4);   % vector part of the injected correction quaternion
    % 
    % S = [           0   -qhat_13(3)   qhat_13(2);
    %       qhat_13(3)             0   -qhat_13(1);
    %      -qhat_13(2)    qhat_13(1)            0 ];
    % 
    % Gamma_q = (eye(3) + S*S)/sqrt(1 - qhat_13'*qhat_13) - S;
    % 
    % G_reset = eye(6);
    % G_reset(1:3,1:3) = Gamma_q;
    % 
    % P_k_k = G_reset * P_k_k * G_reset';
    % P_k_k = 0.5*(P_k_k + P_k_k');
else
    x_k_k = x_k_k1;
    P_k_k = P_k_k1;

    dq = [1;0.5 * x_k_k(1:3)];
    dq = dq/norm(dq);
    
    q_k_k = quatmultiply(q_k_k1',dq')';
    q_k_k = q_k_k/norm(q_k_k);
    x_k_k(1:3) = 0;

end
%%%%%%% star tracker end
% disp(mod(tick,uint64(sun.Ts/Kalman.Ts)))
x_k_k_base = x_k_k;
q_k_k_base = q_k_k;
P_k_k_base = P_k_k;
if any(sun_valid) && mod(tick,uint64(sun.Ts/Kalman.Ts)) == uint64(0) 

    %%%%%%%%%% prev  mag_XYZ_icrf_model,mag_XYZ_body_meas,mag_valid
    P_k_k1 = P_k_k;
    x_k_k1 = x_k_k;
    q_k_k1 = q_k_k;

        %%%valid FSS no
    FSS_no = sum(sun_valid == 1);
    sun_vectors_valid = sun_vectors(:,:,sun_valid);

    %% normalise
    sat2sun_icrf = sat2sun_icrf/norm(sat2sun_icrf);

    for kk = 1:FSS_no
        sun_vectors_valid(:,:,kk) = sun_vectors_valid(:,:,kk)/norm(sun_vectors_valid(:,:,kk));
    end





    %% get current rotation matrix
    q_k_k1;
    dcm_icrf2b = quat2dcm(q_k_k1');

    %%%kalman gain and mearuement error    
    R_k_sun = eye(FSS_no*3) * sun.sigma^2;


    % R_k_sun_one = 10000000 *sun.sigma^2 * (eye(3) - sat2sun_body*sat2sun_body') + 10000000 * sun.sigma^2 * (sat2sun_body*sat2sun_body');
    % R_k_sun = kron(eye(FSS_no), R_k_sun_one);

    sat2sun_body = dcm_icrf2b * sat2sun_icrf;
    sat2sun_body_skew = [0, -sat2sun_body(3), sat2sun_body(2);
        sat2sun_body(3), 0, -sat2sun_body(1);
        -sat2sun_body(2), sat2sun_body(1), 0];

    H_k_row = [sat2sun_body_skew,zeros(3,3),zeros(3,3)];
    H_k = repmat(H_k_row,FSS_no,1);


    K_k = P_k_k1*H_k'/(H_k*P_k_k1*H_k' + R_k_sun);

    % sun_vectors_valid_body = pagemtimes(dcm_icrf2b,sun_vectors_valid);
    y_k = reshape(sun_vectors_valid, [], 1);%%% measurment in body frame stacked already in body pay attention

    h_x_k = repmat(sat2sun_body,FSS_no,1);


    x_k_k = x_k_k1 + K_k * (y_k - h_x_k);
    % 
    P_k_k = (eye(9,9) - K_k * H_k) * P_k_k1 * (eye(9,9) - K_k * H_k)' + K_k * R_k_sun * K_k';

    %% get quat
    dq = [1;0.5 * x_k_k(1:3)];
    dq = dq/norm(dq);

    q_k_k = quatmultiply(q_k_k1',dq')';
    q_k_k = q_k_k/norm(q_k_k);

    x_k_k(1:3) = 0;

    % disp('sun')
    % disp(tick)
    % disp('sun')
else
    % x_k_k = x_k_k;
    % P_k_k = P_k_k;
end
% 
% %%%%%% sun sensor start
if mag_valid && mod(tick,uint64(mag.Ts/Kalman.Ts)) == uint64(0)  

    %%%%%%%%%% prev  mag_XYZ_icrf_model,mag_XYZ_body_meas,mag_valid
    P_k_k1 = P_k_k;
    x_k_k1 = x_k_k;
    q_k_k1 = q_k_k;

    %% normalise
    mag_XYZ_icrf_model = mag_XYZ_icrf_model/norm(mag_XYZ_icrf_model);


    R_k_mag = mag.R_k/norm(mag_XYZ_body_meas)^2;
    % R_k_mag = mag.R_k;

    mag_XYZ_body_meas = mag_XYZ_body_meas/norm(mag_XYZ_body_meas);





    %% get current rotation matrix
    q_k_k1;
    dcm_icrf2b = quat2dcm(q_k_k1');


    %%%kalman gain and mearuement error    



    mag_XYZ_body_model = dcm_icrf2b * mag_XYZ_icrf_model;

    % R_k_sun_one = 10000000 *sun.sigma^2 * (eye(3) - sat2sun_body*sat2sun_body') + 10000000 * sun.sigma^2 * (sat2sun_body*sat2sun_body');
    % R_k_sun = kron(eye(FSS_no), R_k_sun_one);

    mag_XYZ_body_model_skew = [0, -mag_XYZ_body_model(3), mag_XYZ_body_model(2);
        mag_XYZ_body_model(3), 0, -mag_XYZ_body_model(1);
        -mag_XYZ_body_model(2), mag_XYZ_body_model(1), 0];

    H_k_row = [mag_XYZ_body_model_skew,zeros(3,3),zeros(3,3)];
    H_k = H_k_row;


    K_k = P_k_k1*H_k'/(H_k*P_k_k1*H_k' + R_k_mag);

    % sun_vectors_valid_body = pagemtimes(dcm_icrf2b,sun_vectors_valid);
    y_k = mag_XYZ_body_meas;

    h_x_k = mag_XYZ_body_model;


    x_k_k = x_k_k1 + K_k * (y_k - h_x_k);
    % 
    P_k_k = (eye(9,9) - K_k * H_k) * P_k_k1 * (eye(9,9) - K_k * H_k)' + K_k * R_k_mag * K_k';

    %% get quat
    dq = [1;0.5 * x_k_k(1:3)];
    dq = dq/norm(dq);

    q_k_k = quatmultiply(q_k_k1',dq')';
    q_k_k = q_k_k/norm(q_k_k);

    x_k_k(1:3) = 0;

    % 
    % disp('mag')
    % disp(tick)
    % disp('mag')
else
    % x_k_k = x_k_k;
    % P_k_k = P_k_k;
end
% 
% 
% dq = [1;0.5 * x_k_k(1:3)];
% dq = dq/norm(dq);
% 
% q_k_k = quatmultiply(q_k_k1',dq')';
% q_k_k = q_k_k/norm(q_k_k);
% x_k_k_base = x_k_k;
% q_k_k_base = q_k_k;
% P_k_k_base = P_k_k;

x_k_k(1:6) = x_k_k_base(1:6);
q_k_k = q_k_k_base;
% P_k_k(:,1:6) = P_k_k_base(:,1:6);
% P_k_k(1:6,7:9) = P_k_k_base(1:6,7:9);
P_k_k;

x_k_k(1:3) = 0;

q_k_k_state = q_k_k;
x_k_k_state = x_k_k;
P_k_k_state = P_k_k;
tick = tick + 1;