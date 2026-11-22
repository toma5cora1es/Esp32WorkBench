function out = run_preflight_cal(testName, port, boardPath)
%RUN_PREFLIGHT_CAL Staged bench safety & calibration suite for Classroom AGV.
%   Tests: 'macs' | 'rtt' | 'watchdog' | 'motors' | 'deadband' | 'ultrasonic'

    board = wb_config('board', boardPath);
    baud = board.macros.AGV_SERIAL_BAUD;
    cmdAddr = board.registers.commandWordAddr;
    telemAddr = board.registers.telemetryBaseAddr;
    nWords = board.registers.telemetryWordCount;

    esp32_cli('connect', port, baud);
    out = struct();

    switch lower(strtrim(char(testName)))
        case 'macs'
            st = esp32_cli('status', port);
            wb_log('normal', 'CAL', 'MAC', st.ChipStatus);
            out = st;

        case 'rtt'
            n = 200;
            usbMs = zeros(1, n);
            airUs = zeros(1, n);
            ageMs = zeros(1, n);
            esp32_cli('log', 'warn', 'error');
            for k = 1:n
                t0 = tic;
                esp32_cli('write', cmdAddr, pack_cmd(mod(k, 2) + 1, 0, 0, 0, 0));
                esp32_cli('dump', telemAddr, nWords);
                airUs(k) = double(esp32_cli('read', board.registers.linkRttUsAddr));
                ageMs(k) = double(esp32_cli('read', board.registers.linkAgeMsAddr));
                usbMs(k) = toc(t0) * 1000.0;
            end
            esp32_cli('log', 'all');
            sAir = sort(airUs);
            p95AirMs = sAir(ceil(0.95 * numel(sAir))) / 1000.0;
            statsWord = esp32_cli('read', board.registers.linkStatsAddr);
            txOk = double(bitshift(statsWord, -16));
            txFail = double(bitand(statsWord, 65535));
            wb_log('normal', 'CAL', 'RTT', sprintf('USB loop mean=%.1f ms | ESP-NOW air mean=%.2f ms (p95=%.2f ms) | TX ok=%d fail=%d', ...
                mean(usbMs), mean(airUs) / 1000.0, p95AirMs, txOk, txFail));

            figure('Name', 'Pre-Flight RTT & Link Health');
            subplot(2, 1, 1); plot(airUs / 1000.0); ylabel('ESP-NOW Air RTT (ms)'); grid on;
            subplot(2, 1, 2); plot(ageMs); ylabel('Last RX Age (ms)'); xlabel('Packet #'); grid on;
            out = struct('usbMs', usbMs, 'airUs', airUs, 'ageMs', ageMs, 'p95AirMs', p95AirMs, 'txOk', txOk, 'txFail', txFail);

        case 'watchdog'
            esp32_cli('write', cmdAddr, pack_cmd(1, 1, 0, 30, 30));
            pause(0.10);
            d1 = decode_telem(esp32_cli('dump', telemAddr, nWords));
            pause(0.40);
            d2 = decode_telem(esp32_cli('dump', telemAddr, nWords));
            wb_log('normal', 'CAL', 'WATCHDOG', sprintf('Active FSM=%d -> After 400ms silence FSM=%d (exp 0)', d1.fsm, d2.fsm));
            out = struct('before', d1, 'after', d2);

        case 'motors'
            wb_log('warn', 'CAL', 'SAFETY', 'Ensure AGV wheels are OFF THE FLOOR!');
            esp32_cli('write', cmdAddr, pack_cmd(15, 1, 0, 0, 0));
            seq = { 'L_FWD', 40, 0; 'L_REV', -40, 0; 'R_FWD', 0, 40; 'R_REV', 0, -40 };
            res = struct('step', {}, 'dEncL', {}, 'dEncR', {});
            for s = 1:size(seq, 1)
                d0 = decode_telem(esp32_cli('dump', telemAddr, nWords));
                t0 = tic;
                k = 0;
                while toc(t0) < 0.8
                    k = k + 1;
                    esp32_cli('write', cmdAddr, pack_cmd(mod(k, 2) + 1, 1, 0, seq{s, 2}, seq{s, 3}));
                    pause(0.05);
                end
                d1 = decode_telem(esp32_cli('dump', telemAddr, nWords));
                esp32_cli('write', cmdAddr, pack_cmd(1, 0, 0, 0, 0));
                pause(0.25);
                res(s) = struct('step', seq{s, 1}, 'dEncL', d1.encL - d0.encL, 'dEncR', d1.encR - d0.encR);
                wb_log('normal', 'CAL', seq{s, 1}, sprintf('dEncL=%d | dEncR=%d', res(s).dEncL, res(s).dEncR));
            end
            out = res;

        case 'deadband'
            pwms = 10:10:100;
            vel = zeros(size(pwms));
            dEncL = zeros(size(pwms));
            dEncR = zeros(size(pwms));
            esp32_cli('log', 'warn', 'error');
            for i = 1:numel(pwms)
                d0 = decode_telem(esp32_cli('dump', telemAddr, nWords));
                t0 = tic;
                k = 0;
                while toc(t0) < 0.6
                    k = k + 1;
                    esp32_cli('write', cmdAddr, pack_cmd(mod(k, 2) + 1, 1, 0, pwms(i), pwms(i)));
                    pause(0.05);
                end
                d1 = decode_telem(esp32_cli('dump', telemAddr, nWords));
                dEncL(i) = d1.encL - d0.encL;
                dEncR(i) = d1.encR - d0.encR;
                vel(i) = d1.vel_mms;
                esp32_cli('write', cmdAddr, pack_cmd(1, 0, 0, 0, 0));
                pause(0.25);
            end
            esp32_cli('log', 'all');
            figure('Name', 'Deadband & Wheel Balance Sweep');
            plot(pwms, dEncL, '-o', pwms, dEncR, '-s');
            legend('Left Edges/0.6s', 'Right Edges/0.6s'); xlabel('Cmd (%)'); ylabel('Encoder Edges'); grid on;
            out = struct('cmdPct', pwms, 'dEncL', dEncL, 'dEncR', dEncR, 'vel_mms', vel);

        case 'ultrasonic'
            n = 60;
            us = zeros(1, n);
            t = zeros(1, n);
            t0 = tic;
            esp32_cli('log', 'warn', 'error');
            for k = 1:n
                t(k) = toc(t0);
                d = decode_telem(esp32_cli('dump', telemAddr, nWords));
                us(k) = d.us_mm;
                pause(0.04);
            end
            esp32_cli('log', 'all');
            wb_log('normal', 'CAL', 'US', sprintf('Mean=%.1f mm | Std=%.1f mm', mean(us), std(us)));
            figure('Name', 'Filtered HC-SR04 Stability');
            plot(t, us, '-o'); xlabel('Time (s)'); ylabel('Filtered US (mm)'); grid on;
            out = struct('t', t, 'us_mm', us);

        otherwise
            error('CAL:UnknownTest', 'Unknown test "%s".', char(testName));
    end

    esp32_cli('disconnect', port);
end

function w = pack_cmd(magic, mode, aux, l, r)
    w = bitor( ...
        bitor(bitshift(uint32(magic), 28), bitshift(uint32(mode), 24)), ...
        bitor(bitshift(bitand(uint32(aux), 255), 16), bitor(bitshift(uint32(bitand(int32(l), 255)), 8), uint32(bitand(int32(r), 255)))));
end

function d = decode_telem(w)
    d.encL      = i16(bitshift(w(1), -16));
    d.encR      = i16(bitand(w(1), 65535));
    d.us_mm     = double(bitshift(w(2), -16));
    d.lineMask  = double(bitand(bitshift(w(2), -8), 7));
    d.fsm       = double(bitand(w(2), 255));
    d.x_mm      = i16(bitshift(w(3), -16));
    d.y_mm      = i16(bitand(w(3), 65535));
    d.theta_rad = i16(bitshift(w(4), -16)) / 1000.0;
    d.vel_mms   = i16(bitand(w(4), 65535));
end

function v = i16(u)
    v = double(int32(u));
    if v >= 32768, v = v - 65536; end
end