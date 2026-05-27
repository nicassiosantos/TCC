%% =====================================================================
%  rodar_simulink_test.m
%
%  Carrega a marcha do OpenSim, prepara as variaveis do workspace
%  necessarias pelo modelo Simulink, executa a simulacao e plota
%  os resultados.
%
%  Pre-requisitos:
%   - subject01_walk1.mot       (saida da IK Tool)
%   - inverse_dynamics.sto      (saida da ID Tool, opcional aqui)
%   - ler_opensim.m, pegar_coluna.m, preparar_referencia_opensim.m
%   - ortese_idb_pid_simulink.slx   
%
%  Dependencia minima do MATLAB: R2018a (Simulink). Toolboxes:
%   - Simulink, Simulink Control Design.
%% =====================================================================

clear; clc; close all;

%% --- (1) Parametros fisicos -------------------
m  = 4.0;
l_link = 0.40;
r  = 0.05;
d  = l_link/2;
g  = 9.81;
b_visc = 0.5;
D11 = m*l_link^2/3 + m*r^2/4;
mgd = m*g*d;

%% --- (2) Ganhos do PID --------------------------------
zeta_d = 0.8;  wn_d = 8;  alpha_d = 8;
Kp = D11*wn_d^2*(1 + 2*zeta_d*alpha_d);
Ki = D11*alpha_d*wn_d^3;
Kd = D11*wn_d*(2*zeta_d + alpha_d) - b_visc;

fprintf('Ganhos: Kp=%.2f  Ki=%.2f  Kd=%.2f\n\n', Kp, Ki, Kd);

%% --- (3) Configuracao da simulacao ----------------------------------
Ts      = 1e-3;
tau_max = 200;


%% --- (4) Carregar marcha do OpenSim ---------------------------------
arq_mot = 'subject01_walk1.mot';
arq_sto = 'inverse_dynamics.sto';   % deixe '' se quiser pular o sto
coord   = 'knee_angle_r';
fc      = 6;                        % filtro Butterworth biomecanico

M_aux = ler_opensim(arq_mot);
t_orig = M_aux.t;
t_vec  = t_orig(1) : Ts : t_orig(end);

[ref, ref_d, ref_dd, ~] = preparar_referencia_opensim( ...
        arq_mot, '', coord, t_vec, fc);

theta0_lin = mean(ref);
tau_eq     = mgd*cos(theta0_lin);

% Simulink trabalha em tempo comecando em 0
t_sim_vec = t_vec - t_vec(1);
t_end     = t_sim_vec(end);

%% --- (5) Criar timeseries para os blocos From Workspace -------------
ts_theta_ref      = timeseries(ref(:),    t_sim_vec(:));
ts_theta_dot_ref  = timeseries(ref_d(:),  t_sim_vec(:));
ts_theta_ddot_ref = timeseries(ref_dd(:), t_sim_vec(:));

%% --- (6) Condicoes iniciais (casam com o primeiro ponto da marcha) --
theta_init = ref(1);
thd_init   = ref_d(1);

%% --- (7) Garantir que o modelo Simulink exista ----------------------
model = 'ortese_idb_pid_simulink';
if ~exist([model '.slx'], 'file')
    fprintf('Arquivo %s.slx nao encontrado. Criando...\n', model);
    criar_modelo_simulink();
end

%% --- (8) Carregar o modelo e rodar a simulacao ----------------------
if ~bdIsLoaded(model)
    load_system(model);
end

fprintf('Rodando Simulink (t_end = %.3f s, Ts = %.0e s)...\n', t_end, Ts);
sim_out = sim(model, ...
    'StopTime',  num2str(t_end), ...
    'ReturnWorkspaceOutputs', 'on');

%% --- (9) Extrair resultados -----------------------------------------
theta_sim = sim_out.theta_sim;   % timeseries
tau_sim   = sim_out.tau_sim;
err_sim   = sim_out.err_sim;

%% --- (10) Metricas --------------------------------------------------
% Re-amostra theta_sim na malha t_sim_vec para comparar com a referencia
theta_at_ref = interp1(theta_sim.Time, theta_sim.Data, t_sim_vec, ...
                       'linear', 'extrap');
err_deg = rad2deg(ref(:) - theta_at_ref(:));

idx_regime = round(0.1/Ts)+1 : length(t_sim_vec);
rms_total  = rms(err_deg);
rms_regime = rms(err_deg(idx_regime));
pico_err   = max(abs(err_deg));
tau_pk     = max(abs(tau_sim.Data));

fprintf('\n====================================================\n');
fprintf(' Resultado (Simulink, planta nao-linear)\n');
fprintf('====================================================\n');
fprintf('  RMS total  = %.4f deg\n', rms_total);
fprintf('  RMS regime = %.4f deg\n', rms_regime);
fprintf('  Pico erro  = %.4f deg\n', pico_err);
fprintf('  |tau|_max  = %.2f N.m\n', tau_pk);
fprintf('====================================================\n\n');

%% --- (11) Plotagem --------------------------------------------------
figure('Position', [80 80 1000 720], 'Name','Simulink IDB+PID');

subplot(3,1,1);
plot(t_sim_vec, rad2deg(ref), 'k--', 'LineWidth', 1.4); hold on;
plot(theta_sim.Time, rad2deg(theta_sim.Data), 'b', 'LineWidth', 1.1);
ylabel('\theta [graus]');
legend('Referencia OpenSim', 'Simulink IDB+PID', 'Location', 'best');
title(sprintf(['Simulink IDB+PID -- Planta NL, marcha do OpenSim\n' ...
               'K_p=%.1f, K_i=%.1f, K_d=%.1f (mesmos do degrau)'], ...
               Kp, Ki, Kd));
grid on;

subplot(3,1,2);
plot(t_sim_vec, err_deg, 'r');
ylabel('Erro [graus]');
title(sprintf('Erro de rastreio -- RMS regime = %.4f deg', rms_regime));
grid on;

subplot(3,1,3);
plot(tau_sim.Time, tau_sim.Data, 'b');
yline( tau_max, 'r:', 'LineWidth', 0.8);
yline(-tau_max, 'r:', 'LineWidth', 0.8);
ylabel('\tau [N\cdotm]');
xlabel('Tempo [s]');
title(sprintf('Torque aplicado (saturacao em \\pm %d N\\cdotm)', tau_max));
grid on;

%% --- (12) Salvar resultados -----------------------------------------
save('resultados_simulink.mat', ...
     't_sim_vec', 'ref', 'theta_sim', 'tau_sim', 'err_sim', ...
     'rms_total', 'rms_regime', 'pico_err', 'tau_pk', ...
     'Kp', 'Ki', 'Kd');
fprintf('Resultados salvos em resultados_simulink.mat\n');