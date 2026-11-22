
function checks = run_wire_audit(dutPort, simPort, boardPath)
%RUN_WIRE_AUDIT Physical GPIO jumper verification between Board A (DUT) and Board B (SIM).
%   Uses core/esp32_cli directly with zero custom twin_cli wrapper.

    board = wb_config('board', boardPath);
    cmdAddr = board.registers.commandWordAddr;
    telemAddr = board.registers.telemetryBaseAddr;
    telemWords = board.registers.telemetryWordCount;

    esp32_cli('connect', dutPort);
    esp32_cli('connect', simPort);

    wb_log('normal', 'AUDIT', 'START', sprintf('DUT=%s <-> SIM=%s (%s)', upper(dutPort), upper(simPort), board.boardName));

    % Set SIM virtual environment: LineMask=101 (5), Dist=350mm
    simCmd = bitor(bitshift(uint32(5), 16), uint32(350));
    esp32_cli('on', simPort, 'write', cmdAddr, simCmd);

    % Drive DUT forward at 60% L/R with both Front+Rear LEDs ON (aux=3)
    dutCmd = pack_dut_word(1, 1, 3, 60, 60);
    esp32_cli('on', dutPort, 'write', cmdAddr, dutCmd);
    pause(0.30);

    simW = esp32_cli('on', simPort, 'dump', telemAddr, telemWords);
    dutW = esp32_cli('on', dutPort, 'dump', telemAddr, telemWords);

    % Return DUT to E-STOP
    esp32_cli('on', dutPort, 'write', cmdAddr, pack_dut_word(1, 0, 0, 0, 0));

    sim = decode_sim_words(simW);
    dut = decode_dut_words(dutW);

    b = board.macros;
    checks = {
        sprintf('Motor L PWM (A:D%d -> B:D%d)', b.AGV_PIN_MOT_L_A, b.SIM_MOT_LA_IN), abs(sim.pwmL - 60) <= 5 && sim.freqL > 1500, sprintf('%d%% @ %d Hz', sim.pwmL, sim.freqL);
        sprintf('Motor R PWM (A:D%d -> B:D%d)', b.AGV_PIN_MOT_R_A, b.SIM_MOT_RA_IN), abs(sim.pwmR - 60) <= 5 && sim.freqR > 1500, sprintf('%d%% @ %d Hz', sim.pwmR, sim.freqR);
        'LEDs (FL/FR/RR/RL -> B)',                                                   sim.ledMask == 15,                         sprintf('Mask=0x%X (exp 0xF)', sim.ledMask);
        sprintf('Line L/M/R (B:%d,%d,%d -> A:%d,%d,%d)', b.SIM_LINE_L_OUT, b.SIM_LINE_M_OUT, b.SIM_LINE_R_OUT, b.AGV_PIN_LINE_L, b.AGV_PIN_LINE_M, b.AGV_PIN_LINE_R), dut.lineMask == 5, sprintf('Read=%s (exp 101)', dec2bin(dut.lineMask, 3));
        sprintf('Ultrasonic (D%d Trig / D%d Echo)', b.AGV_PIN_US_TRIG, b.AGV_PIN_US_ECHO), sim.trigCount > 0 && abs(dut.us_mm - 350) <= 35, sprintf('Trig=%d Meas=%d mm (exp 350)', sim.trigCount, dut.us_mm)
    };

    for k = 1:size(checks, 1)
        if checks{k, 2}
            wb_log('normal', 'AUDIT', checks{k, 1}, ['PASS | ' checks{k, 3}]);
        else
            wb_log('error', 'AUDIT', checks{k, 1}, ['FAIL | ' checks{k, 3}]);
        end
    end

    esp32_cli('disconnect', dutPort);
    esp32_cli('disconnect', simPort);
end

function w = pack_dut_word(magic, mode, aux, l, r)
    w = bitor( ...
        bitor(bitshift(uint32(magic), 28), bitshift(uint32(mode), 24)), ...
        bitor(bitshift(bitand(uint32(aux), 255), 16), bitor(bitshift(uint32(bitand(int32(l), 255)), 8), uint32(bitand(int32(r), 255)))));
end

function d = decode_dut_words(w)
    d.encL     = i16(bitshift(w(1), -16));
    d.encR     = i16(bitand(w(1), 65535));
    d.us_mm    = double(bitshift(w(2), -16));
    d.lineMask = double(bitand(bitshift(w(2), -8), 7));
    d.fsm      = double(bitand(w(2), 255));
end

function s = decode_sim_words(w)
    s.pwmL      = i16(bitshift(w(1), -16));
    s.pwmR      = i16(bitand(w(1), 65535));
    s.freqL     = double(bitshift(w(2), -16));
    s.freqR     = double(bitand(w(2), 65535));
    s.ledMask   = double(bitand(bitshift(w(3), -16), 15));
    s.trigCount = double(bitand(w(3), 65535));
end

function v = i16(u)
    v = double(int32(u));
    if v >= 32768
        v = v - 65536;
    end
end