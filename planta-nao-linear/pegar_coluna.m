function col = pegar_coluna(S, nome)
%PEGAR_COLUNA  Recupera uma coluna nomeada de S.data.
%
%   col = pegar_coluna(S, 'knee_angle_r') retorna o vetor coluna
%   correspondente a coordenada `knee_angle_r` da struct S
%   produzida por ler_opensim().
%
%   Lanca erro com lista das colunas disponiveis se o nome
%   nao for encontrado.

    idx = find(strcmpi(S.cols, nome));
    if isempty(idx)
        error(['Coluna "%s" nao encontrada em S.\n' ...
               'Disponiveis: %s'], nome, strjoin(S.cols, ', '));
    end
    col = S.data(:, idx);
end