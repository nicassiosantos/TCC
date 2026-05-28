%% =====================================================================
%  rodar_simulink_linear_test.m
%
%  Roda a simulacao Simulink IDB+PID com a PLANTA LINEARIZADA em torno
%  de theta0_lin = mean(theta_ref), sob a marcha real do OpenSim.
%
%  Diferenca em relacao ao rodar_simulink_test.m:
%   - Calcula matrizes A, B, C, D da linearizacao
%   - Define tau_eq_lin = mgd*cos(theta0_lin) (torque de equilibrio)
%   - Inicial state x_init_lin em variaveis de desvio
%% =====================================================================

clear; clc; close all;

%% --- Parametros fisicos ---
m  = 4.0;  l_link = 0.40;  r = 0.05;  d = l_link/2;
g  = 9.81; b_visc = 0.5;
D11 = m*l_link^2/3 + m*r^2/4;
mgd = m*g*d;

%% --- Ganhos do PID (invariantes) ---
zeta_d = 0.8; wn_d = 8; alpha_d = 8;
Kp = D11*wn_d^2*(1 + 2*zeta_d*alpha_d);
Ki = D11*alpha_d*wn_d^3;
Kd = D11*wn_d*(2*zeta_d + alpha_d) - b_visc;
fprintf('Ganhos: Kp=%.2f  Ki=%.2f  Kd=%.2f\n\n', Kp, Ki, Kd);

%% --- Periodo de amostragem e saturacao ---
Ts      = 1e-3;
tau_max = 200;

%% --- Carregar marcha do OpenSim ---
arq_mot = 'subject01_walk1.mot';
coord   = 'knee_angle_r';
fc      = 6;

M_aux = ler_opensim(arq_mot);
t_orig = M_aux.t;
t_vec  = t_orig(1) : Ts : t_orig(end);

[ref, ref_d, ref_dd, ~] = preparar_referencia_opensim( ...
        arq_mot, '', coord, t_vec, fc);

t_sim_vec = t_vec - t_vec(1);
t_end     = t_sim_vec(end);

%% --- Timeseries para os blocos From Workspace ---
ts_theta_ref      = timeseries(ref(:),    t_sim_vec(:));
ts_theta_dot_ref  = timeseries(ref_d(:),  t_sim_vec(:));
ts_theta_ddot_ref = timeseries(ref_dd(:), t_sim_vec(:));

%% --- LINEARIZACAO ---
% Linearizamos em torno do centro da marcha para que a aproximacao
% seja valida durante toda a trajetoria.
theta0_lin = mean(ref);
tau_eq_lin = mgd*cos(theta0_lin);

% Matrizes A, B, C, D (consistente com o relatorio de linearizacao)
A_mat = [0,                          1;
         mgd*sin(theta0_lin)/D11,   -b_visc/D11];
B_mat = [0; 1/D11];
C_mat = [1, 0];
D_mat = 0;

% Estado inicial (em variaveis de desvio)
x_init_lin = [ref(1) - theta0_lin;   % theta_tilde(0)
              ref_d(1)];             % theta_dot_tilde(0) = theta_dot(0)

fprintf('--- Linearizacao ---\n');
fprintf('  theta0_lin = %.2f deg\n', rad2deg(theta0_lin));
fprintf('  tau_eq_lin = %.3f N.m\n', tau_eq_lin);
fprintf('  A = [%.4f %.4f; %.4f %.4f]\n', A_mat(1,1),A_mat(1,2),A_mat(2,1),A_mat(2,2));
fprintf('  B = [%.4f; %.4f]\n', B_mat(1), B_mat(2));
fprintf('  Polos da malha aberta: %s\n\n', mat2str(eig(A_mat), 4));

%% --- Garantir modelo Simulink ---
model = 'ortese_idb_pid_simulink_linear';
if ~exist([model '.slx'], 'file')
    fprintf('Modelo %s.slx nao encontrado. Criando...\n', model);
    criar_modelo_simulink_linear();
end
if ~bdIsLoaded(model)
    load_system(model);
end

%% --- Simular ---
fprintf('Rodando Simulink (planta linear)...\n');
sim_out = sim(model, ...
    'StopTime', num2str(t_end), ...
    'ReturnWorkspaceOutputs', 'on');

theta_sim = sim_out.theta_sim;
tau_sim   = sim_out.tau_sim;
err_sim   = sim_out.err_sim;

%% --- Metricas ---
theta_at_ref = interp1(theta_sim.Time, theta_sim.Data, t_sim_vec, 'linear', 'extrap');
err_deg = rad2deg(ref(:) - theta_at_ref(:));

idx_regime = round(0.1/Ts)+1 : length(t_sim_vec);
rms_total  = rms(err_deg);
rms_regime = rms(err_deg(idx_regime));
pico_err   = max(abs(err_deg));
tau_pk     = max(abs(tau_sim.Data));

fprintf('\n====================================================\n');
fprintf(' Resultado (Simulink, planta LINEAR)\n');
fprintf('====================================================\n');
fprintf('  RMS total  = %.4f deg\n', rms_total);
fprintf('  RMS regime = %.4f deg\n', rms_regime);
fprintf('  Pico erro  = %.4f deg\n', pico_err);
fprintf('  |tau|_max  = %.2f N.m\n', tau_pk);
fprintf('====================================================\n\n');

%% --- Plotagem ---
figure('Position', [80 80 1000 720], 'Name', 'Simulink IDB+PID -- Planta LINEAR');

subplot(3,1,1);
plot(t_sim_vec, rad2deg(ref), 'k--', 'LineWidth', 1.4); hold on;
plot(theta_sim.Time, rad2deg(theta_sim.Data), 'r', 'LineWidth', 1.1);
ylabel('\theta [graus]');
legend('Referencia OpenSim', 'Simulink IDB+PID (LIN)', 'Location', 'best');
title(sprintf(['Simulink IDB+PID -- Planta LINEAR (\\theta_0=%.1f deg)\n' ...
               'K_p=%.1f, K_i=%.1f, K_d=%.1f (mesmos ganhos da NL)'], ...
               rad2deg(theta0_lin), Kp, Ki, Kd));
grid on;

subplot(3,1,2);
plot(t_sim_vec, err_deg, 'r');
ylabel('Erro [graus]');
title(sprintf('Erro -- RMS regime = %.4f deg', rms_regime));
grid on;

subplot(3,1,3);
plot(tau_sim.Time, tau_sim.Data, 'r');
yline( tau_max, 'k:'); yline(-tau_max, 'k:');
ylabel('\tau [N\cdotm]'); xlabel('Tempo [s]');
grid on;

save('resultados_simulink_linear.mat', ...
     't_sim_vec', 'ref', 'theta_sim', 'tau_sim', 'err_sim', ...
     'rms_total', 'rms_regime', 'pico_err', 'tau_pk', ...
     'A_mat', 'B_mat', 'C_mat', 'D_mat', 'theta0_lin', 'tau_eq_lin');