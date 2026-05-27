function S = ler_opensim(filename)
%LER_OPENSIM  Le arquivo .mot ou .sto do OpenSim.
%
%   S = ler_opensim('arquivo.mot') devolve uma struct S com:
%       S.t          - vetor de tempo (segundos)
%       S.cols       - cell array com os nomes das colunas (sem 'time')
%       S.data       - matriz numerica, cada coluna = uma variavel
%       S.in_degrees - flag do cabecalho ('yes' -> true)
%
%   Para obter uma coluna especifica, use pegar_coluna.m:
%       knee = pegar_coluna(S, 'knee_angle_r');
%       if S.in_degrees, knee = deg2rad(knee); end
%
%   Funciona tanto para .mot (cinematica, de IK Tool) quanto .sto
%   (torques, de ID Tool). A diferenca esta apenas nas unidades:
%   .mot tem angulos em graus (se inDegrees=yes), .sto tem torques
%   em N.m e nao se aplica conversao.

    fid = fopen(filename, 'r');
    if fid == -1
        error('Nao consegui abrir o arquivo: %s', filename);
    end

    % --- ler cabecalho ate 'endheader' ---
    S.in_degrees = false;
    while true
        line = fgetl(fid);
        if ~ischar(line), break; end
        line = strtrim(line);
        if startsWith(lower(line), 'indegrees=yes')
            S.in_degrees = true;
        elseif startsWith(lower(line), 'indegrees=no')
            S.in_degrees = false;
        elseif strcmpi(line, 'endheader')
            break;
        end
    end

    % --- ler nomes das colunas (primeira linha nao vazia apos endheader) ---
    while true
        line = fgetl(fid);
        if ischar(line) && ~isempty(strtrim(line)), break; end
    end
    headers = strsplit(strtrim(line));   % primeira col = 'time'

    % --- ler dados numericos ---
    fmt = repmat('%f', 1, length(headers));
    raw = textscan(fid, fmt, 'CollectOutput', true);
    fclose(fid);
    matriz = raw{1};

    S.t    = matriz(:, 1);
    S.data = matriz(:, 2:end);
    S.cols = headers(2:end);
end