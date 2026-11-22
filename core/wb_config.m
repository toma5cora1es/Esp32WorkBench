function [cfg, resolvedPath] = wb_config(section, arg)
%WB_CONFIG Direct JSON loader (always reads fresh from disk; zero stale cache).

    if nargin < 1 || isempty(section)
        wb_log('error', 'CONFIG', 'MISSING_SECTION', 'Specify config section: workbench, instruments, or board');
        error('WB:MissingConfigSection', 'Specify config section: workbench | instruments | board.');
    end

    key = lower(strtrim(char(section)));

    switch key
        case 'workbench'
            resolvedPath = fullfile(wb_paths('config'), 'workbench_config.json');
            cfg = jsondecode(fileread(resolvedPath));

        case 'instruments'
            resolvedPath = fullfile(wb_paths('config'), 'instruments.json');
            cfg = jsondecode(fileread(resolvedPath));

        case 'board'
            resolvedPath = wb_paths('board', arg);
            cfg = jsondecode(fileread(resolvedPath));
            if ~isfield(cfg, 'boardName') || ~isfield(cfg, 'macros')
                wb_log('error', 'CONFIG', 'BAD_BOARD', sprintf('Board JSON missing boardName/macros: %s', resolvedPath));
                error('WB:BadBoardSchema', 'Board JSON missing required fields: %s', resolvedPath);
            end

        otherwise
            wb_log('error', 'CONFIG', 'BAD_SECTION', sprintf('Unknown config section "%s"', key));
            error('WB:BadConfigSection', 'Unknown config section "%s".', key);
    end
end