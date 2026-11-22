function varargout = esp32_cli(cmd, varargin)
%ESP32_CLI Core ESP32 CLI with explicit single/multi-board port targeting ('on', port, ...).

    if nargin == 0 || strcmpi(strtrim(char(cmd)), 'help')
        print_help();
        return;
    end

    targetPort = '';
    act = lower(strtrim(char(cmd)));
    args = varargin;

    if strcmp(act, 'on')
        if numel(args) < 2
            wb_log('error', 'CLI', 'ON_SYNTAX', 'Usage: esp32_cli(''on'', port, subcmd, ...)');
            error('ESP32:MissingArgs', 'Usage: esp32_cli(''on'', port, subcmd, ...)');
        end
        targetPort = upper(strtrim(char(args{1})));
        act = lower(strtrim(char(args{2})));
        args = args(3:end);
    end

    switch act
        case 'log'
            if isempty(args)
                lvls = wb_log('get_level');
                print_log_status(lvls);
                if nargout > 0, varargout{1} = lvls; end
            elseif strcmpi(strtrim(char(args{1})), 'history')
                hist = wb_log('history');
                if nargout > 0, varargout{1} = hist; end
            else
                lvls = wb_log('set_level', args{:});
                print_log_status(lvls);
                if nargout > 0, varargout{1} = lvls; end
            end

        case 'scan'
            ports = esp32_io('scan');
            if nargout > 0
                varargout{1} = ports;
            end

        case 'connect'
            ok = esp32_io('connect', args{:});
            if nargout > 0
                varargout{1} = ok;
            end

        case 'disconnect'
            if ~isempty(targetPort)
                ok = esp32_io('disconnect', targetPort);
            else
                ok = esp32_io('disconnect', args{:});
            end
            if nargout > 0
                varargout{1} = ok;
            end

        case 'status'
            out = exec_status(targetPort, args{:});
            if nargout > 0
                varargout{1} = out;
            end

        case 'compile'
            rep = esp32_flash('compile', args{:});
            if nargout > 0
                varargout{1} = rep;
            end

        case 'flash'
            rep = esp32_flash('flash', args{:});
            if nargout > 0
                varargout{1} = rep;
            end

        case 'sfr'
            if isempty(args)
                out = wb_sfr('query', '');
            elseif strcmpi(strtrim(char(args{1})), 'compile')
                out = wb_sfr('compile');
            else
                out = wb_sfr('query', args{1});
            end
            if nargout > 0
                varargout{1} = out;
            end

        case 'read'
            [vals, t] = exec_read(targetPort, nargout == 0, args{:});
            if nargout > 0, varargout{1} = vals; end
            if nargout > 1, varargout{2} = t; end

        case 'write'
            rb = exec_write(targetPort, args{1}, args{2});
            if nargout > 0
                varargout{1} = rb;
            end

        case 'dump'
            vals = exec_dump(targetPort, nargout == 0, args{1}, args{2});
            if nargout > 0
                varargout{1} = vals;
            end

        case 'bits'
            out = exec_bits(targetPort, args{:});
            if nargout > 0
                varargout{1} = out;
            end

        case 'cmd'
            if numel(args) >= 2 && isnumeric(args{2})
                count = double(args{2});
                rawCmd = char(args{1});
                resps = strings(1, count);
                t = zeros(1, count);
                wb_log('normal', 'CLI', 'CMD_BURST', sprintf('"%s" x%d', rawCmd, count));
                t0 = tic;
                for k = 1:count
                    t(k) = toc(t0);
                    resps(k) = string(io_cmd(targetPort, rawCmd));
                end
                if nargout > 0, varargout{1} = resps; end
                if nargout > 1, varargout{2} = t; end
            else
                resp = io_cmd(targetPort, char(args{1}));
                if nargout > 0
                    varargout{1} = resp;
                end
            end

        otherwise
            wb_log('error', 'CLI', 'UNKNOWN_CMD', sprintf('Unknown command "%s". Run esp32_cli(''help'').', act));
            error('ESP32:UnknownCommand', 'Unknown command "%s".', act);
    end
end

function [resp, rtt] = io_cmd(targetPort, rawCmd)
    if isempty(targetPort)
        [resp, rtt] = esp32_io('cmd', rawCmd);
    else
        [resp, rtt] = esp32_io('cmd', targetPort, rawCmd);
    end
end

function out = exec_status(targetPort, varargin)
    if isempty(targetPort) && ~isempty(varargin)
        targetPort = upper(strtrim(char(varargin{1})));
    end

    if ~isempty(targetPort)
        s = esp32_io('state', targetPort);
        if ~s.Connected
            wb_log('warn', 'CLI', 'STATUS', sprintf('%s is disconnected', targetPort));
            out = s;
            return;
        end
        [chipResp, rtt] = esp32_io('cmd', targetPort, 'STATUS');
        s.ChipStatus = chipResp;
        s.LastLatencyMs = rtt;
        wb_log('normal', 'CLI', 'STATUS', sprintf('%s (%d baud) | RTT=%.1f ms | %s', s.Target, s.Baud, rtt, chipResp));
        out = s;
        return;
    end

    s = esp32_io('state');
    if ~s.Connected
        wb_log('warn', 'CLI', 'STATUS', 'No ports connected');
        out = s;
        return;
    end

    ports = s.OpenPorts;
    out = struct([]);
    for k = 1:numel(ports)
        p = ports{k};
        st = esp32_io('state', p);
        [chipResp, rtt] = esp32_io('cmd', p, 'STATUS');
        st.ChipStatus = chipResp;
        st.LastLatencyMs = rtt;
        wb_log('normal', 'CLI', 'STATUS', sprintf('%s (%d baud) | RTT=%.1f ms | %s', st.Target, st.Baud, rtt, chipResp));
        out = [out, st]; %#ok<AGROW>
    end
end

function [vals, t] = exec_read(targetPort, showFields, target, count)
    isVectorTarget = (isstring(target) && ~isscalar(target)) || iscell(target) || (isnumeric(target) && ~isscalar(target));

    if nargin >= 4 && isnumeric(count)
        n = double(count);
        [addr, sym, ~] = wb_sfr('resolve', target);

        lbl = format_reg_label(sym, addr);
        wb_log('normal', 'REG', 'SAMPLE_START', sprintf('%s x%d', lbl, n));

        vals = zeros(1, n, 'uint32');
        t = zeros(1, n);
        cmdStr = sprintf('R %08X', addr);
        t0 = tic;

        for k = 1:n
            t(k) = toc(t0);
            vals(k) = parse_fw_hex32(io_cmd(targetPort, cmdStr));
        end

        wb_log('normal', 'REG', 'SAMPLE_DONE', sprintf('%s: %d samples in %.3fs', lbl, n, t(end)));
        return;
    end

    if isVectorTarget
        n = numel(target);
        vals = zeros(1, n, 'uint32');
        t = zeros(1, n);
        t0 = tic;

        for k = 1:n
            if iscell(target)
                item = target{k};
            else
                item = target(k);
            end
            [addr, sym, ~] = wb_sfr('resolve', item);
            t(k) = toc(t0);
            vals(k) = parse_fw_hex32(io_cmd(targetPort, sprintf('R %08X', addr)));
            wb_log('normal', 'REG', 'READ', sprintf('%s = 0x%08X (%u)', format_reg_label(sym, addr), vals(k), vals(k)));
        end
        return;
    end

    [addr, sym, rm] = wb_sfr('resolve', target);
    vals = parse_fw_hex32(io_cmd(targetPort, sprintf('R %08X', addr)));
    t = [];

    wb_log('normal', 'REG', 'READ', sprintf('%s = 0x%08X (%u) | %s', format_reg_label(sym, addr), vals, vals, bin32(vals)));
    if showFields && ~isempty(rm)
        wb_sfr('print_fields', rm.bitfields, vals);
    end
end

function rb = exec_write(targetPort, target, valIn)
    [addr, sym, ~] = wb_sfr('resolve', target);

    nv = parse_user_u32(valIn);
    rb = parse_fw_hex32(io_cmd(targetPort, sprintf('W %08X %08X', addr, nv)));
    lbl = format_reg_label(sym, addr);

    if rb ~= nv
        wb_log('warn', 'REG', 'MISMATCH', sprintf('Wrote 0x%08X to %s, read back 0x%08X', nv, lbl, rb));
    else
        wb_log('normal', 'REG', 'WRITE', sprintf('%s <= 0x%08X (RB=0x%08X)', lbl, nv, rb));
    end
end

function vals = exec_dump(targetPort, showTable, target, count)
    n = double(count);
    [baseAddr, sym, ~] = wb_sfr('resolve', target);
    resp = io_cmd(targetPort, sprintf('D %08X %d', baseAddr, n));
    tokens = strsplit(strtrim(resp));

    vals = zeros(1, numel(tokens), 'uint32');
    for k = 1:numel(tokens)
        vals(k) = parse_fw_hex32(tokens{k});
    end

    wb_log('normal', 'REG', 'DUMP', sprintf('%s x%d words', format_reg_label(sym, baseAddr), numel(vals)));
    if showTable
        for k = 1:numel(vals)
            addr = baseAddr + uint32((k - 1) * 4);
            fprintf(1, '    0x%08X : 0x%08X | %s\n', addr, vals(k), bin32(vals(k)));
        end
    end
end

function out = exec_bits(targetPort, target, fieldName, valIn)
    [addr, sym, rm] = wb_sfr('resolve', target);
    bf = wb_sfr('bitfield', rm, fieldName);

    lsb = double(bf.offset);
    width = double(bf.width);
    mask = bitshift(uint32(2^width - 1), lsb);
    cur = parse_fw_hex32(io_cmd(targetPort, sprintf('R %08X', addr)));
    uField = upper(strtrim(char(fieldName)));

    if nargin < 4
        fv = bitshift(bitand(cur, mask), -lsb);
        wb_log('normal', 'REG', 'BITS_READ', sprintf('%s.%s = %u (0x%X)', sym, uField, fv, fv));
        out = fv;
        return;
    end

    if strcmpi(bf.access, 'R')
        wb_log('warn', 'REG', 'READ_ONLY', sprintf('%s.%s is marked read-only (R)', sym, uField));
    end

    nf = bitand(parse_user_u32(valIn), uint32(2^width - 1));
    nextWord = bitor(bitand(cur, bitcmp(mask, 'uint32')), bitshift(nf, lsb));
    rb = parse_fw_hex32(io_cmd(targetPort, sprintf('W %08X %08X', addr, nextWord)));

    wb_log('normal', 'REG', 'BITS_WRITE', sprintf('%s.%s <= %u -> 0x%08X', sym, uField, nf, rb));
    out = rb;
end

function n = parse_user_u32(in)
    if isnumeric(in)
        n = uint32(in);
        return;
    end

    s = strtrim(char(in));
    if startsWith(s, '0x', 'IgnoreCase', true)
        hexPart = s(3:end);
        if isempty(hexPart) || ~all(isstrprop(hexPart, 'xdigit'))
            wb_log('error', 'CLI', 'PARSE_U32', sprintf('Invalid hex uint32: "%s"', s));
            error('ESP32:InvalidU32', 'Invalid hex uint32: "%s".', s);
        end
        n = uint32(hex2dec(hexPart));
        return;
    end

    if isempty(s) || ~all(isstrprop(s, 'digit'))
        wb_log('error', 'CLI', 'PARSE_U32', sprintf('Expected decimal digits or 0xHEX (got "%s")', s));
        error('ESP32:InvalidU32', 'Expected decimal digits or 0xHEX (got "%s").', s);
    end
    n = uint32(str2double(s));
end

function n = parse_fw_hex32(resp)
    s = regexprep(strtrim(char(resp)), '^0[xX]', '');
    if isempty(s) || ~all(isstrprop(s, 'xdigit'))
        wb_log('error', 'CLI', 'FW_HEX', sprintf('Expected hex word from firmware, got "%s"', char(resp)));
        error('ESP32:InvalidFirmwareResponse', 'Expected hex word from firmware, got "%s".', char(resp));
    end
    n = uint32(hex2dec(s));
end

function lbl = format_reg_label(sym, addr)
    if strlength(sym) > 0
        lbl = sprintf('%s (0x%08X)', sym, addr);
    else
        lbl = sprintf('0x%08X', addr);
    end
end

function b = bin32(v)
    r = dec2bin(v, 32);
    b = sprintf('%s_%s_%s_%s_%s_%s_%s_%s', r(1:4), r(5:8), r(9:12), r(13:16), r(17:20), r(21:24), r(25:28), r(29:32));
end

function print_log_status(lvls)
    if isempty(lvls)
        fprintf(1, '  [LOG] Enabled: NONE\n');
    else
        fprintf(1, '  [LOG] Enabled: %s\n', upper(strjoin(lvls, ', ')));
    end
end

function print_help()
    fprintf(1, '\n==================== ESP32 WORKBENCH CLI ====================\n');
    fprintf(1, '  log [''all'' | ''off'' | ''normal'' | ''warn'' | ''error'' | ''history'']\n');
    fprintf(1, '  scan | connect(port, [baud]) | disconnect([port|''all'']) | status([port])\n');
    fprintf(1, '  rep = compile(''file''|''inline'', source, [''board'',..., ''outDir'',..., ''partitions'',...])\n');
    fprintf(1, '  rep = flash(''file''|''inline'', source, port, [''board'',..., ''outDir'',..., ''partitions'',...])\n');
    fprintf(1, '  sfr [query | ''compile'']\n');
    fprintf(1, '  val = read(regOr0xHex, [count]) | rb = write(regOr0xHex, decOr0xHex)\n');
    fprintf(1, '  vec = dump(baseRegOr0xHex, count) | val = bits(reg, field, [newVal]) | cmd(rawCmd, [count])\n');
    fprintf(1, '  Multi-board prefix: esp32_cli(''on'', ''COM8'', subcmd, ...)\n');
    fprintf(1, '=============================================================\n\n');
end