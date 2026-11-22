function out = wb_paths(kind, name)
%WB_PATHS Repository path structure and explicit path resolution.

    rootEnv = getenv('ESP32_WB_ROOT');
    if ~isempty(rootEnv) && isfolder(rootEnv)
        rootDir = rootEnv;
    else
        rootDir = fileparts(fileparts(mfilename('fullpath')));
    end

    p = struct( ...
        'root',          rootDir, ...
        'config',        fullfile(rootDir, 'config'), ...
        'boards',        fullfile(rootDir, 'config', 'boards'), ...
        'core',          fullfile(rootDir, 'core'), ...
        'firmware',      fullfile(rootDir, 'firmware'), ...
        'lib',           fullfile(rootDir, 'firmware', 'lib'), ...
        'templates',     fullfile(rootDir, 'firmware', 'templates'), ...
        'databases',     fullfile(rootDir, 'databases'), ...
        'build',         fullfile(rootDir, 'build'), ...
        'sfrDb',         fullfile(rootDir, 'databases', 'esp32_sfr.json'));

    if nargin == 0 || isempty(kind)
        out = p;
        return;
    end

    switch lower(strtrim(char(kind)))
        case 'board'
            out = resolve_path(p, name);
            if ~isfile(out)
                wb_log('error', 'PATHS', 'NO_BOARD', sprintf('Board file not found: %s', char(name)));
                error('WB:MissingBoard', 'Board file not found: %s', char(name));
            end
        case 'sketch'
            out = resolve_sketch(p, name);
        case 'resolve'
            out = resolve_path(p, name);
        case 'rel'
            out = to_relative(p.root, name);
        case 'discover'
            out = discover_assets(p, name);
        otherwise
            out = p.(char(kind));
    end
end

function resolved = resolve_path(p, rawIn)
    s = strrep(strrep(strtrim(char(rawIn)), '/', filesep), '\', filesep);
    if isfile(s) || isfolder(s)
        resolved = s;
        return;
    end
    resolved = fullfile(p.root, s);
end

function resolved = resolve_sketch(p, rawIn)
    cand = resolve_path(p, rawIn);
    if isfile(cand)
        resolved = cand;
        return;
    end

    if isfolder(cand)
        [~, folderName] = fileparts(cand);
        ino = fullfile(cand, [folderName '.ino']);
        if isfile(ino)
            resolved = ino;
            return;
        end
    end

    wb_log('error', 'PATHS', 'NO_SKETCH', sprintf('Sketch file not found: %s', char(rawIn)));
    error('ESP32:SketchNotFound', 'Sketch file not found: %s', char(rawIn));
end

function rel = to_relative(rootDir, absPath)
    rel = strrep(char(absPath), '\', '/');
    rootNorm = strrep(rootDir, '\', '/');
    if ~endsWith(rootNorm, '/')
        rootNorm = [rootNorm '/'];
    end
    if startsWith(rel, rootNorm, 'IgnoreCase', true)
        rel = extractAfter(rel, strlength(rootNorm));
    end
end

function list = discover_assets(p, kind)
    switch lower(strtrim(char(kind)))
        case 'boards'
            d = dir(fullfile(p.boards, '*.json'));
        case 'templates'
            d = dir(fullfile(p.templates, '*.ino'));
        otherwise
            error('WB:BadDiscoverKind', 'Unknown discovery kind: %s', char(kind));
    end

    list = strings(1, numel(d));
    for k = 1:numel(d)
        list(k) = string(to_relative(p.root, fullfile(d(k).folder, d(k).name)));
    end
end