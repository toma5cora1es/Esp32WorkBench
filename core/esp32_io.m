function [out, latencyMs] = esp32_io(action, varargin)
%ESP32_IO Multi-port fail-fast Serial transport for ESP32 workbench.

    persistent conns
    if isempty(conns)
        conns = struct();
    end

    out = [];
    latencyMs = 0;

    if nargin < 1 || isempty(action)
        action = 'state';
    end

    act = lower(strtrim(char(action)));

    switch act
        case 'state'
            if ~isempty(varargin) && ~isempty(varargin{1})
                target = upper(strtrim(char(varargin{1})));
                key = port_key(target);
                if isfield(conns, key)
                    out = conns.(key);
                else
                    out = struct('Connected', false, 'Target', target, 'Baud', 0, 'SerialObj', [], 'Busy', false, 'LastLatencyMs', 0);
                end
                return;
            end

            keys = fieldnames(conns);
            openPorts = cell(1, numel(keys));
            for k = 1:numel(keys)
                openPorts{k} = conns.(keys{k}).Target;
            end

            if numel(keys) == 1
                out = conns.(keys{1});
                out.OpenPorts = openPorts;
            else
                out = struct('Connected', ~isempty(keys), 'Target', '', 'OpenPorts', {openPorts}, 'Connections', conns);
            end

        case 'scan'
            ports = serialportlist("available");
            if isempty(ports)
                wb_log('warn', 'IO', 'SCAN', 'No available serial ports found');
            else
                wb_log('normal', 'IO', 'SCAN', sprintf('Found %d port(s): %s', numel(ports), strjoin(ports, ', ')));
            end
            out = ports(:).';

        case 'connect'
            if isempty(varargin) || isempty(varargin{1})
                wb_log('error', 'IO', 'MISSING_PORT', 'Explicit serial port is required (e.g. COM9)');
                error('ESP32:MissingPort', 'Specify serial port explicitly (e.g. COM9).');
            end

            target = upper(strtrim(char(varargin{1})));
            key = port_key(target);

            if isfield(conns, key)
                wb_log('error', 'IO', 'ALREADY_OPEN', sprintf('%s is already connected', target));
                error('ESP32:AlreadyConnected', '%s is already connected.', target);
            end

            cfg = wb_config('workbench');
            baud = cfg.transport.serial.defaultBaud;
            if numel(varargin) >= 2 && ~isempty(varargin{2})
                baud = double(varargin{2});
            end

            stale = serialportfind('Port', target);
            if ~isempty(stale)
                wb_log('warn', 'IO', 'STALE_HANDLE', sprintf('Closing orphaned MATLAB handle on %s', target));
                delete(stale);
            end

            wb_log('normal', 'IO', 'CONNECT', sprintf('Opening %s at %d baud', target, baud));
            sp = serialport(target, baud, 'Timeout', cfg.transport.serial.connectTimeoutSec);
            configureTerminator(sp, "LF");

            pause(cfg.transport.serial.bootDrainWaitSec);
            pending = sp.NumBytesAvailable;
            if pending > 0
                read(sp, pending, 'uint8');
                wb_log('warn', 'IO', 'BOOT_DRAIN', sprintf('Drained %d boot bytes on %s', pending, target));
            end

            [resp, latencyMs] = tx_rx_serial(sp, target, 'STATUS');
            if ~contains(resp, 'ESP32_READY')
                delete(sp);
                wb_log('error', 'IO', 'HANDSHAKE', sprintf('Unexpected STATUS response on %s: "%s"', target, resp));
                error('ESP32:BadHandshake', 'Expected ESP32_READY on %s, got "%s".', target, resp);
            end

            conns.(key) = struct( ...
                'Connected', true, ...
                'Target', target, ...
                'Baud', baud, ...
                'SerialObj', sp, ...
                'Busy', false, ...
                'LastLatencyMs', latencyMs);

            wb_log('normal', 'IO', 'CONNECTED', sprintf('%s ready (%.1f ms)', target, latencyMs));
            out = true;

        case 'disconnect'
            keys = fieldnames(conns);
            if isempty(keys)
                wb_log('warn', 'IO', 'DISCONNECT', 'No active connections');
                out = false;
                return;
            end

            if ~isempty(varargin) && strcmpi(strtrim(char(varargin{1})), 'all')
                for k = 1:numel(keys)
                    close_entry(conns.(keys{k}));
                end
                conns = struct();
                out = true;
                return;
            end

            explicit = '';
            if ~isempty(varargin) && ~isempty(varargin{1})
                explicit = varargin{1};
            end
            key = resolve_port_key(conns, explicit);
            close_entry(conns.(key));
            conns = rmfield(conns, key);
            out = true;

        case 'cmd'
            if numel(varargin) >= 2
                explicit = varargin{1};
                rawCmd = strtrim(char(varargin{2}));
            else
                explicit = '';
                rawCmd = strtrim(char(varargin{1}));
            end

            key = resolve_port_key(conns, explicit);
            entry = conns.(key);

            if entry.Busy
                wb_log('error', 'IO', 'BUSY', sprintf('%s transport is busy', entry.Target));
                error('ESP32:TransportBusy', '%s is already executing a command.', entry.Target);
            end

            conns.(key).Busy = true;
            try
                [out, latencyMs] = tx_rx_serial(entry.SerialObj, entry.Target, rawCmd);
                conns.(key).LastLatencyMs = latencyMs;
                conns.(key).Busy = false;
            catch ME
                conns.(key).Busy = false;
                wb_log('error', 'IO', 'CMD_FAIL', sprintf('%s "%s" failed: %s', entry.Target, rawCmd, ME.message));
                rethrow(ME);
            end

        otherwise
            wb_log('error', 'IO', 'BAD_ACTION', sprintf('Unknown esp32_io action: %s', act));
            error('ESP32:BadIoAction', 'Unknown esp32_io action: %s', act);
    end
end

function key = port_key(target)
    key = matlab.lang.makeValidName(upper(strtrim(char(target))));
end

function key = resolve_port_key(conns, explicit)
    keys = fieldnames(conns);
    if isempty(keys)
        wb_log('error', 'IO', 'NOT_CONNECTED', 'No ports connected');
        error('ESP32:NotConnected', 'No ports connected. Call esp32_cli(''connect'', port) first.');
    end

    if ~isempty(explicit)
        key = port_key(explicit);
        if ~isfield(conns, key)
            wb_log('error', 'IO', 'NOT_CONNECTED', sprintf('Port %s is not connected', upper(char(explicit))));
            error('ESP32:NotConnected', 'Port %s is not connected.', upper(char(explicit)));
        end
        return;
    end

    if numel(keys) > 1
        ports = cell(1, numel(keys));
        for k = 1:numel(keys)
            ports{k} = conns.(keys{k}).Target;
        end
        wb_log('error', 'IO', 'AMBIGUOUS', sprintf('Multiple ports open (%s); specify port explicitly', strjoin(ports, ', ')));
        error('ESP32:AmbiguousTarget', 'Multiple ports connected (%s). Specify target via esp32_cli(''on'', port, ...).', strjoin(ports, ', '));
    end

    key = keys{1};
end

function close_entry(entry)
    if ~isempty(entry.SerialObj) && isvalid(entry.SerialObj)
        flush(entry.SerialObj);
        delete(entry.SerialObj);
    end
    wb_log('normal', 'IO', 'DISCONNECT', sprintf('Closed %s', entry.Target));
end

function [out, ms] = tx_rx_serial(sp, target, rawCmd)
    wb_log('normal', 'IO', 'TX', sprintf('[%s] %s', target, rawCmd));
    t0 = tic;

    pending = sp.NumBytesAvailable;
    if pending > 0
        extra = strtrim(char(read(sp, pending, 'char')));
        wb_log('warn', 'IO', 'UNEXPECTED_RX', sprintf('[%s] Discarded %d bytes before "%s": %s', target, pending, rawCmd, extra));
    end

    writeline(sp, rawCmd);
    out = strtrim(char(readline(sp)));
    if isempty(out)
        wb_log('error', 'IO', 'TIMEOUT', sprintf('Serial timeout on %s for "%s"', target, rawCmd));
        error('ESP32:SerialTimeout', 'Serial read timed out on %s for "%s".', target, rawCmd);
    end

    ms = toc(t0) * 1000.0;
    wb_log('normal', 'IO', 'RX', sprintf('[%s] (%.1f ms) %s', target, ms, out));
end