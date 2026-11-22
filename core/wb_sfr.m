function varargout = wb_sfr(action, varargin)
%WB_SFR ESP32 Special Function Register database compiler, resolver, and query.

    persistent dbCache

    if nargin < 1 || isempty(action)
        action = 'load';
    end

    switch lower(strtrim(char(action)))
        case 'load'
            if isempty(dbCache)
                p = wb_paths('sfrDb');
                if ~isfile(p)
                    wb_log('error', 'SFR', 'MISSING_DB', sprintf('Missing %s; run esp32_cli(''sfr'', ''compile'') first', wb_paths('rel', p)));
                    error('SFR:DatabaseMissing', 'SFR database missing. Run esp32_cli(''sfr'', ''compile'').');
                end
                dbCache = jsondecode(fileread(p));
                wb_log('normal', 'SFR', 'LOAD', sprintf('%d registers (%s)', numel(fieldnames(dbCache.registers)), wb_paths('rel', p)));
            end
            varargout{1} = dbCache;

        case 'compile'
            p = wb_paths('sfrDb');
            dbCache = compile_sfr_db(p);
            varargout{1} = dbCache;

        case 'resolve'
            [addr, sym, rm] = resolve_reg(varargin{1});
            varargout{1} = addr;
            varargout{2} = sym;
            varargout{3} = rm;

        case 'bitfield'
            rm = varargin{1};
            fieldName = upper(strtrim(char(varargin{2})));
            if isempty(rm) || ~isfield(rm, 'bitfields') || ~isfield(rm.bitfields, fieldName)
                wb_log('error', 'SFR', 'NO_BITFIELD', sprintf('Bitfield "%s" not available on target register', fieldName));
                error('SFR:InvalidBitfield', 'Bitfield "%s" not found on register.', fieldName);
            end
            varargout{1} = rm.bitfields.(fieldName);

        case 'query'
            q = '';
            if ~isempty(varargin)
                q = upper(strtrim(char(varargin{1})));
            end
            varargout{1} = query_sfr(q);

        case 'print_fields'
            print_fields(varargin{1}, varargin{2});

        otherwise
            wb_log('error', 'SFR', 'BAD_ACTION', sprintf('Unknown wb_sfr action: %s', action));
            error('SFR:BadAction', 'Unknown wb_sfr action: %s', action);
    end
end

function [addr, sym, rm] = resolve_reg(in)
    db = wb_sfr('load');
    sym = "";
    rm = [];

    if isnumeric(in)
        addr = uint32(in);
        return;
    end

    u = upper(strtrim(char(in)));
    if isfield(db.registers, u)
        sym = string(u);
        rm = db.registers.(u);
        addr = uint32(hex2dec(regexprep(char(rm.address), '^0X', '')));
        return;
    end

    if startsWith(u, '0X')
        hexPart = u(3:end);
        if ~isempty(hexPart) && all(isstrprop(hexPart, 'xdigit'))
            addr = uint32(hex2dec(hexPart));
            return;
        end
    end

    wb_log('error', 'SFR', 'UNKNOWN_REG', sprintf('Expected SFR symbol or 0xHEX address (got "%s")', u));
    error('SFR:UnknownRegister', 'Register "%s" not found (use exact SFR name or 0xHEX).', u);
end

function out = query_sfr(q)
    db = wb_sfr('load');
    regs = string(fieldnames(db.registers));

    if isempty(q)
        wb_log('normal', 'SFR', 'SUMMARY', sprintf('%d registers available', numel(regs)));
        out = regs(:).';
        return;
    end

    if isfield(db.registers, q)
        rm = db.registers.(q);
        fprintf(1, '\n  === %s (%s) ===\n', q, rm.address);
        bfs = fieldnames(rm.bitfields);
        for k = 1:numel(bfs)
            bf = rm.bitfields.(bfs{k});
            fprintf(1, '    %-28s [%2d:%2d] %s\n', bfs{k}, bf.offset + bf.width - 1, bf.offset, bf.access);
        end
        fprintf(1, '\n');
        out = rm;
        return;
    end

    matches = regs(contains(regs, q, 'IgnoreCase', true));
    if isempty(matches)
        wb_log('warn', 'SFR', 'NO_MATCH', sprintf('No registers match "%s"', q));
        out = strings(1, 0);
        return;
    end

    wb_log('normal', 'SFR', 'QUERY', sprintf('%d match(es) for "%s"', numel(matches), q));
    for k = 1:min(numel(matches), 40)
        rName = matches(k);
        fprintf(1, '    %-34s %s\n', rName, db.registers.(rName).address);
    end
    if numel(matches) > 40
        fprintf(1, '    ... (%d more)\n', numel(matches) - 40);
    end

    out = matches(:).';
end

function print_fields(bfs, val)
    names = fieldnames(bfs);
    for k = 1:numel(names)
        bf = bfs.(names{k});
        mask = bitshift(uint32(2^double(bf.width) - 1), double(bf.offset));
        fv = bitshift(bitand(val, mask), -double(bf.offset));
        if fv ~= 0
            fprintf(1, '    %-26s = 0x%X (%u)\n', names{k}, fv, fv);
        end
    end
end

function db = compile_sfr_db(outJsonPath)
    cfg = wb_config('workbench');
    sdkRoot = expand_env(char(cfg.toolchain.sdkPackagesRoot));

    hits = dir(fullfile(sdkRoot, '**', 'soc', 'esp32', 'register', 'soc', 'gpio_reg.h'));
    if isempty(hits)
        wb_log('error', 'SFR', 'NO_HEADERS', sprintf('ESP32 SDK register headers not found under %s', sdkRoot));
        error('SFR:MissingHeaders', 'ESP32 SDK register headers not found under %s', sdkRoot);
    end
    socRegDir = hits(1).folder;

    socH = fullfile(socRegDir, 'soc.h');
    if ~isfile(socH)
        wb_log('error', 'SFR', 'NO_SOC_H', sprintf('Missing soc.h at %s', socH));
        error('SFR:MissingSocHeader', 'Missing soc.h at %s', socH);
    end

    baseMap = containers.Map('KeyType', 'char', 'ValueType', 'uint32');
    toks = regexp(fileread(socH), '#define\s+(DR_REG_\w+_BASE)\s+\(?0x([0-9a-fA-F]+)\)?', 'tokens');
    for k = 1:numel(toks)
        baseMap(toks{k}{1}) = uint32(hex2dec(toks{k}{2}));
    end

    if baseMap.Count == 0
        wb_log('error', 'SFR', 'EMPTY_BASES', sprintf('Failed to parse DR_REG_*_BASE macros from %s', socH));
        error('SFR:EmptyBaseMap', 'Failed to parse base addresses from %s', socH);
    end

    regFiles = dir(fullfile(socRegDir, '*_reg.h'));
    registers = struct();

    for f = 1:numel(regFiles)
        lines = splitlines(string(fileread(fullfile(regFiles(f).folder, regFiles(f).name))));
        curReg = '';

        for k = 1:numel(lines)
            ln = lines(k);
            mBase = regexp(ln, '^\s*#define\s+([A-Z0-9_]+_REG)\s+\(\s*(DR_REG_\w+_BASE)\s*\+\s*0x([0-9a-fA-F]+)\s*\)', 'tokens', 'once');
            mAbs  = regexp(ln, '^\s*#define\s+([A-Z0-9_]+_REG)\s+\(?0x([0-9a-fA-F]{8})\)?', 'tokens', 'once');

            if ~isempty(mBase) && isKey(baseMap, mBase{2})
                curReg = char(mBase{1});
                addr = baseMap(mBase{2}) + uint32(hex2dec(mBase{3}));
                registers.(curReg) = struct('address', sprintf('0x%08X', addr), 'bitfields', struct());
                continue;
            end

            if ~isempty(mAbs)
                curReg = char(mAbs{1});
                registers.(curReg) = struct('address', ['0x' upper(char(mAbs{2}))], 'bitfields', struct());
                continue;
            end

            if ~isempty(curReg)
                mBf = regexp(ln, '/\*\s*([A-Z0-9_]+)\s*:\s*([A-Z/]+)\s*;bitpos:\[(\d+):(\d+)\]', 'tokens', 'once');
                if ~isempty(mBf)
                    acc = 'RW';
                    if upper(mBf{2}) == "RO"
                        acc = 'R';
                    elseif upper(mBf{2}) == "WO"
                        acc = 'W';
                    end
                    msb = str2double(mBf{3});
                    lsb = str2double(mBf{4});
                    registers.(curReg).bitfields.(char(mBf{1})) = struct('offset', lsb, 'width', msb - lsb + 1, 'access', acc);
                end
            end
        end
    end

    gpioRegs = ["GPIO_OUT_REG", "GPIO_OUT_W1TS_REG", "GPIO_OUT_W1TC_REG", ...
                "GPIO_ENABLE_REG", "GPIO_ENABLE_W1TS_REG", "GPIO_ENABLE_W1TC_REG", "GPIO_IN_REG"];
    for r = gpioRegs
        if isfield(registers, r)
            acc = 'RW';
            if contains(r, '_IN_')
                acc = 'R';
            elseif contains(r, '_W1T')
                acc = 'W';
            end
            for b = 0:31
                registers.(r).bitfields.(sprintf('GPIO%d', b)) = struct('offset', b, 'width', 1, 'access', acc);
            end
        end
    end

    outDir = fileparts(outJsonPath);
    if ~isfolder(outDir)
        mkdir(outDir);
    end

    db = struct('derivative', 'ESP32_D0WD_V3', 'registers', registers);
    fid = fopen(outJsonPath, 'w', 'n', 'UTF-8');
    fwrite(fid, jsonencode(db, 'PrettyPrint', true), 'char');
    fclose(fid);

    wb_log('normal', 'SFR', 'COMPILED', sprintf('%d registers -> %s', numel(fieldnames(registers)), wb_paths('rel', outJsonPath)));
end

function s = expand_env(s)
    toks = regexp(s, '%([^%]+)%', 'tokens');
    for k = 1:numel(toks)
        v = getenv(toks{k}{1});
        if ~isempty(v)
            s = strrep(s, ['%' toks{k}{1} '%'], v);
        end
    end
end