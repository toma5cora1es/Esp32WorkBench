function out = wb_log(action, varargin)
%WB_LOG Unified 3-level plain-text workbench logger (all levels enabled by default).

    persistent state
    if isempty(state)
        state = struct( ...
            'Enabled', struct('normal', true, 'warn', true, 'error', true), ...
            'History', strings(0, 1));
    end

    act = lower(strtrim(char(action)));

    if strcmp(act, 'set_level')
        state.Enabled = parse_enabled_levels(varargin{:});
        out = enabled_list(state.Enabled);
        return;
    end

    if strcmp(act, 'get_level')
        out = enabled_list(state.Enabled);
        return;
    end

    if strcmp(act, 'history')
        fprintf(1, '\n=== LOG HISTORY (%d entries) ===\n', numel(state.History));
        for k = 1:numel(state.History)
            fprintf(1, '  %s\n', state.History(k));
        end
        fprintf(1, '================================\n\n');
        out = state.History;
        return;
    end

    if ~ismember(act, {'normal', 'warn', 'error'})
        error('WB:InvalidLogLevel', 'Log call level must be normal, warn, or error (got "%s").', act);
    end

    modTag = char(varargin{1});
    stageTag = char(varargin{2});
    msg = char(varargin{3});

    ts = char(datetime('now', 'Format', 'HH:mm:ss.SSS'));
    line = sprintf('[%s] [%-6s] [%-8s] %-14s | %s', ts, upper(act), upper(modTag), stageTag, msg);

    state.History(end + 1) = string(line);
    if numel(state.History) > 300
        state.History = state.History(end - 299:end);
    end

    if ~state.Enabled.(act)
        out = false;
        return;
    end

    if strcmp(act, 'error')
        fprintf(2, '  %s\n', line);
    else
        fprintf(1, '  %s\n', line);
    end

    out = true;
end

function en = parse_enabled_levels(varargin)
    tokens = strings(0, 1);
    for k = 1:numel(varargin)
        arg = varargin{k};
        if iscell(arg)
            for c = 1:numel(arg)
                tokens = [tokens; split_tokens(arg{c})]; %#ok<AGROW>
            end
        else
            arr = string(arg);
            for c = 1:numel(arr)
                tokens = [tokens; split_tokens(arr(c))]; %#ok<AGROW>
            end
        end
    end

    tokens = lower(strtrim(tokens));
    tokens(strlength(tokens) == 0) = [];

    if isempty(tokens)
        error('WB:InvalidLogLevel', 'Specify all, off, or any combination of normal, warn, error.');
    end

    if numel(tokens) == 1 && strcmp(tokens(1), 'all')
        en = struct('normal', true, 'warn', true, 'error', true);
        return;
    end

    if numel(tokens) == 1 && any(strcmp(tokens(1), {'off', 'none'}))
        en = struct('normal', false, 'warn', false, 'error', false);
        return;
    end

    en = struct('normal', false, 'warn', false, 'error', false);
    for k = 1:numel(tokens)
        t = char(tokens(k));
        if ~ismember(t, {'normal', 'warn', 'error'})
            error('WB:InvalidLogLevel', 'Invalid log level "%s". Use all, off, normal, warn, or error.', t);
        end
        en.(t) = true;
    end
end

function toks = split_tokens(val)
    parts = regexp(char(val), '[\s,;]+', 'split');
    toks = string(parts(:));
end

function list = enabled_list(en)
    names = ["normal", "warn", "error"];
    keep = [en.normal, en.warn, en.error];
    list = names(keep);
end