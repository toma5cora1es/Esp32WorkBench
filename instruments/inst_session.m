function out = inst_session(action, arg1, arg2)
%INST_SESSION Explicit visadev SCPI instrument session helper.

    switch lower(strtrim(char(action)))
        case 'open'
            instCfg = wb_config('instruments');
            key = char(arg1);
            meta = instCfg.(key);
            addr = char(meta.visaAddress);
            if nargin >= 3 && ~isempty(arg2)
                addr = char(arg2);
            end

            vdev = visadev(addr);
            vdev.Timeout = double(meta.timeoutMs) / 1000.0;
            configureTerminator(vdev, 'LF');

            out = struct('Dev', vdev, 'Addr', addr, 'Key', key);
            out.Idn = inst_session('query', out, '*IDN?');
            wb_log('normal', 'VISA', upper(key), out.Idn);

        case 'write'
            writeline(arg1.Dev, char(arg2));
            out = true;

        case 'query'
            out = strtrim(char(writeread(arg1.Dev, char(arg2))));

        case 'close'
            if isvalid(arg1.Dev)
                delete(arg1.Dev);
            end
            out = true;

        otherwise
            wb_log('error', 'VISA', 'BAD_ACTION', sprintf('Unknown inst_session action: %s', char(action)));
            error('INST:UnknownAction', 'Unknown inst_session action: %s', char(action));
    end
end