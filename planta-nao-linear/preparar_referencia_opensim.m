function [ref, ref_d, ref_dd, tau_FF_OS] = preparar_referencia_opensim( ...
        arquivo_mot, arquivo_sto, coord, t_target, fc)
%PREPARAR_REFERENCIA_OPENSIM
%   Constroi theta_ref(t), theta_dot_ref(t), theta_ddot_ref(t) e
%   tau_FF_OS(t) na grade temporal t_target, a partir dos arquivos
%   do OpenSim.
%
%   Entradas:
%       arquivo_mot - caminho do .mot da Inverse Kinematics
%       arquivo_sto - caminho do .sto da Inverse Dynamics
%                     (pode ser '' se nao usar IDB do OpenSim)
%       coord       - nome da coordenada (ex: 'knee_angle_r')
%       t_target    - vetor de tempo da simulacao (Ts = 1 ms)
%       fc          - frequencia de corte do Butterworth (Hz)
%                     (recomendado: 6 Hz, padrao biomecanico)
%
%   Saidas:
%       ref         - theta_ref(t)        em radianos
%       ref_d       - theta_dot_ref(t)    em rad/s
%       ref_dd      - theta_ddot_ref(t)   em rad/s^2
%       tau_FF_OS   - torque do OpenSim   em N.m  (ou [] se sem .sto)

    % --- 1) Le o .mot e extrai a coordenada do joelho ---
    M = ler_opensim(arquivo_mot);
    theta_os = pegar_coluna(M, coord);
    if M.in_degrees
        theta_os = deg2rad(theta_os);
    end

    % --- 2) Filtra com Butterworth de 4a ordem (zero-phase) ---
    fs = 1/mean(diff(M.t));               % freq. de amostragem original
    [bL, aL] = butter(4, fc/(fs/2));
    theta_filt = filtfilt(bL, aL, theta_os);

    % --- 3) Reamostra para a grade alvo (Ts da simulacao) ---
    ref = interp1(M.t, theta_filt, t_target, 'spline', 'extrap');

    % --- 4) Deriva numericamente (gradient = central differences) ---
    Ts = mean(diff(t_target));
    ref_d  = gradient(ref,  Ts);
    ref_dd = gradient(ref_d, Ts);

    % --- 5) Re-filtra as derivadas (ruido amplificado por derivacao) ---
    fs2 = 1/Ts;
    [bD, aD] = butter(4, fc/(fs2/2));
    ref_d  = filtfilt(bD, aD, ref_d);
    ref_dd = filtfilt(bD, aD, ref_dd);

    % --- 6) Le tau do OpenSim (se solicitado) ---
    if isempty(arquivo_sto)
        tau_FF_OS = [];
    else
        I = ler_opensim(arquivo_sto);
        % Coluna do torque do joelho: '<coord>_moment'
        nome_torque = [coord '_moment'];
        tau_os = pegar_coluna(I, nome_torque);
        tau_filt = filtfilt(bL, aL, tau_os);   % mesmo filtro do angulo
        tau_FF_OS = interp1(I.t, tau_filt, t_target, 'spline', 'extrap');
    end
end