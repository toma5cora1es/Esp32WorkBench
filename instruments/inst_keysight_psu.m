function out = inst_keysight_psu(action, ks, varargin)
%INST_KEYSIGHT_PSU SCPI driver for Keysight E36312A Triple-Channel Power Supply.

    switch lower(strtrim(char(action)))
        case 'open'
            addr = '';
            if nargin >= 2 && ~isempty(ks)
                addr = ks;
            end
            out = inst_session('open', 'keysight_e36312a', addr);
            inst_session('write', out, '*CLS');

        case 'config'
            ch = varargin{1};
            v = varargin{2};
            iLim = varargin{3};
            onState = 'OFF';
            if varargin{4}
                onState = 'ON';
            end
            inst_session('write', ks, sprintf('VOLT %.3f, (@%d)', v, ch));
            inst_session('write', ks, sprintf('CURR %.3f, (@%d)', iLim, ch));
            inst_session('write', ks, sprintf('OUTP %s, (@%d)', onState, ch));
            out = true;

        case 'volt'
            inst_session('write', ks, sprintf('VOLT %.3f, (@%d)', varargin{2}, varargin{1}));
            out = true;

        case 'measure'
            ch = varargin{1};
            v = str2double(inst_session('query', ks, sprintf('MEAS:VOLT? (@%d)', ch)));
            i = str2double(inst_session('query', ks, sprintf('MEAS:CURR? (@%d)', ch)));
            out = struct('V', v, 'I_A', i, 'I_mA', i * 1000.0, 'P_mW', v * i * 1000.0);

        case 'safe_off'
            cfg = wb_config('instruments');
            chs = cfg.keysight_e36312a.channels;
            names = fieldnames(chs);
            for k = 1:numel(names)
                c = chs.(names{k});
                if isfield(c, 'safeOff') && strcmpi(c.safeOff, 'nominal')
                    inst_session('write', ks, sprintf('VOLT %.3f, (@%d)', c.nominalV, c.ch));
                else
                    inst_session('write', ks, sprintf('OUTP OFF, (@%d)', c.ch));
                end
            end
            out = true;

        case 'close'
            inst_keysight_psu('safe_off', ks);
            inst_session('close', ks);
            out = true;

        otherwise
            error('INST:BadAction', 'Unknown inst_keysight_psu action: %s', char(action));
    end
end