ast_c = [1.679 -0.1089 0];
ast_t = [25.56 +0.6761 0];

% sccm_m_conversion_factors = [-0.6761 25.56*2];
sccm_m_conversion_factors = [1.0341 25.531*2];
sccm_m_conversion_factors_c = [0.1089 1.679*2];


p2s = linspace(0.25,0.8,1000);


set_points_sccm = [1.8,2,4,5.7];
set_points_p2 =  ((sccm_m_conversion_factors(1)) +...
sqrt((sccm_m_conversion_factors(1)) ^ (2)  + ((2 .* sccm_m_conversion_factors(2)) .* (set_points_sccm))))...
./ (sccm_m_conversion_factors(2));

sccm_m_conversion_factors = [-0.6761 25.56*2];

sccm_t = [2,2.36,4.3,5.85];
sccm_c = [0.1541,0.167,0.357,0.57];

% set_points_p2 = [0.3,0.39,0.484];
% mb_t = [49.6,72,89.2];
% mb_c = [24,39,48.5,56.1,71];
% sccm_c = [0.1,0.2,0.3,0.4,0.6];
% sccm_t = [2,4,6];
% 
% 
% sccm_t = interp1(mb_t,sccm_t,[53.8,71.4,89]);
% sccm_c = interp1(mb_c,sccm_c,[33.1,47.7,63]);
% 
real_c = polyfit([set_points_p2,0,0,0],[sccm_c,0,0,0],2);
real_t = polyfit([set_points_p2,0,0,0],[sccm_t,0,0,0],2);



figure
hold on
ylabel('sccm')
xlabel('internal set point')
title('Thruster Mass Flow (sccm)','FontSize', 30)
plot(p2s, polyval(ast_t, p2s), 'LineWidth', 2)
plot(p2s, polyval(real_t, p2s), 'LineWidth', 2)
scatter(set_points_p2, sccm_t, 60, 'filled')
legend('AST', 'Experiment', 'FontSize', 24, 'Location', 'best')
set(gca, 'FontSize', 24)
box on

figure
hold on
ylabel('sccm')
xlabel('internal set point')
title('Cathode Mass Flow (sccm)','FontSize', 30)
plot(p2s, polyval(ast_c, p2s), 'LineWidth', 2)
plot(p2s, polyval(real_c, p2s), 'LineWidth', 2)
scatter(set_points_p2, sccm_c, 60, 'filled')

legend('AST', 'Experiment', 'FontSize', 24, 'Location', 'best')
set(gca, 'FontSize', 24)
box on

figure
hold on
title('Split Ratio','FontSize', 30)
ylabel('split ratio')
xlabel('internal set point')
plot(p2s, polyval(ast_t, p2s) ./ polyval(ast_c, p2s), 'LineWidth', 2)
plot(p2s, polyval(real_t, p2s) ./ polyval(real_c, p2s), 'LineWidth', 2)
legend('AST', 'Experiment', 'FontSize', 24, 'Location', 'best')
set(gca, 'FontSize', 24)
box on

% set_points_p2 = [0.3,0.39,0.484];
% mb_t = [49.6,72,89.2];
% mb_c = [24,39,48.5,56.1,71];
% sccm_c = [0.1,0.2,0.3,0.4,0.6];
% sccm_t = [2,4,6];
% 
% 
% sccm_t = interp1(mb_t,sccm_t,[53.8,71.4,89]);
% sccm_c = interp1(mb_c,sccm_c,[33.1,47.7,63]);
% 
% real_c = polyfit([set_points_p2,0,0,0],[sccm_c,0,0,0],2);
% real_t = polyfit([set_points_p2,0,0,0],[sccm_t,0,0,0],2);
% 
% 
% figure(1)
% hold on
% plot(p2s, polyval(real_t, p2s), 'LineWidth', 2)
% scatter(set_points_p2, sccm_t, 60, 'filled')
% legend('AST', 'Exp 1','data 1','Exp - High cathode back pressure','data - High cathode back pressure', 'FontSize', 24, 'Location', 'best')
% 
% figure(2)
% hold on
% plot(p2s, polyval(real_c, p2s), 'LineWidth', 2)
% scatter(set_points_p2, sccm_c, 60, 'filled')
% scatter(0.387,0.288,240,'x',LineWidth=3);
% 
% legend('AST', 'Exp 1','data 1','Exp - High cathode back pressure','data - High cathode back pressure','Acceptance test data', 'FontSize', 24, 'Location', 'best')
% 
% figure(3)
% hold on
% plot(p2s, polyval(real_t, p2s) ./ polyval(real_c, p2s), 'LineWidth', 2)
legend('AST', 'Exp 1','Exp - High cathode back pressure', 'FontSize', 24, 'Location', 'best')

% your data
% x = x(:);
% y = y(:);
% x = double(xx(:));
% y = double(yy(:));
% 
% idx = isfinite(x) & isfinite(y);
% x = x(idx);
% y = y(idx);
% 
% % optional but recommended: shift x so it starts near zero
% x0 = min(x);
% xs = x - x0;
% 
% ft = fittype('a + b*exp(-k*x)', ...
%     'independent', 'x', ...
%     'coefficients', {'a','b','k'});
% 
% a0 = max(y);
% b0 = max(y) - min(y);
% k0 = 1 / max(range(xs), eps);
% 
% [f,gof] = fit(xs, y, ft, ...
%     'StartPoint', [a0 b0 k0], ...
%     'Lower',      [-Inf 0 0], ...
%     'Upper',      [ Inf Inf Inf]);
% 
% figure
% plot(f, xs, y)
% xlabel('x - min(x)')
% ylabel('y')
% legend('Data','Fit')