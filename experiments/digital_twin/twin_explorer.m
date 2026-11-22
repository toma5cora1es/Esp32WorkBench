function twin_explorer(dutPort, simPort, mapPath, boardPath, revealTruth)
%TWIN_EXPLORER Blind-Discovery Hardware Digital Twin built directly on core/esp32_cli.

    if nargin < 5
        revealTruth = false;
    end

    prev = findall(0, 'Type', 'figure', 'Tag', 'TWIN_EXPLORER');
    if ~isempty(prev)
        close(prev);
        drawnow;
    end

    board = wb_config('board', boardPath);
    cmdAddr = board.registers.commandWordAddr;
    telemAddr = board.registers.telemetryBaseAddr;
    telemWords = board.registers.telemetryWordCount;
    usOffsetM = board.kinematics.ultrasonicForwardOffsetMm / 1000.0;

    mapFile = wb_paths('resolve', mapPath);
    if ~isfile(mapFile)
        error('TWIN:MissingMap', 'Map JSON not found: %s', char(mapPath));
    end

    esp32_cli('connect', dutPort);
    esp32_cli('connect', simPort);

    % Zero DUT odometry via magic nibble 0xF
    esp32_cli('on', dutPort, 'write', cmdAddr, pack_dut(15, 1, 0, 0, 0));

    [world, logOdds, axCoords, nCells, halfM, resM] = load_world_map(mapFile);
    st = struct('Running', true, 'Mode', 1, 'Aux', 0, 'Speed', 65, 'CmdL', 0, 'CmdR', 0, 'Magic', 1, ...
                'RevealTruth', logical(revealTruth), 'KeyDriving', false, 'Keys', struct(), ...
                'PoseTrail', zeros(0, 2), 'Hits', zeros(0, 2), 'VelHist', zeros(0, 2), 'T0', tic, 'ErrCount', 0);
    busyTick = false;

    bg = [0.08 0.10 0.14];
    pnl = [0.06 0.08 0.11];
    fig = figure('Name', sprintf('Green AGV Digital Twin (%s <-> %s)', upper(dutPort), upper(simPort)), ...
        'NumberTitle', 'off', 'MenuBar', 'none', 'ToolBar', 'none', 'Color', bg, 'Position', [40 40 1320 740], ...
        'Tag', 'TWIN_EXPLORER', 'WindowKeyPressFcn', @onKeyDown, 'WindowKeyReleaseFcn', @onKeyUp, 'CloseRequestFcn', @onClose);

    pMap = uipanel(fig, 'Title', 'Blind Ultrasonic Discovery Map', 'Units', 'normalized', ...
        'Position', [0.01 0.03 0.62 0.94], 'BackgroundColor', pnl, 'ForegroundColor', [0 0.95 1], 'FontWeight', 'bold');
    ax = axes('Parent', pMap, 'Units', 'normalized', 'Position', [0.08 0.09 0.88 0.86], ...
        'Color', [0.03 0.04 0.07], 'XColor', 'w', 'YColor', 'w');
    hold(ax, 'on'); axis(ax, 'equal'); xlim(ax, [-halfM halfM]); ylim(ax, [-halfM halfM]); grid(ax, 'on');
    xlabel(ax, 'Room X (m) — [WASD/Arrows=Drive | L=Line PID | Space=E-STOP | R=Reset | G=Truth]', 'Color', 'w');
    ylabel(ax, 'Room Y (m)', 'Color', 'w');

    hGrid = imagesc(ax, axCoords, axCoords, logodds_to_prob(logOdds));
    set(ax, 'YDir', 'normal');
    cmap = [linspace(0.10, 0.03, 128)' linspace(0.24, 0.04, 128)' linspace(0.30, 0.07, 128)'; ...
            linspace(0.03, 0.98, 128)' linspace(0.04, 0.92, 128)' linspace(0.07, 0.55, 128)'];
    colormap(ax, cmap); caxis(ax, [0 1]);

    visTruth = 'off';
    if st.RevealTruth, visTruth = 'on'; end
    hTruthGrp = hggroup('Parent', ax, 'Visible', visTruth);
    draw_truth_overlay(hTruthGrp, world);

    hTrail = plot(ax, NaN, NaN, '-', 'Color', [0.15 0.72 1.0], 'LineWidth', 2.0, 'DisplayName', 'DUT Odometry');
    hHits  = scatter(ax, NaN, NaN, 28, [1 0.22 0.25], 'filled', 'DisplayName', 'HW Ultrasonic Hits');
    hRay   = plot(ax, NaN, NaN, '--', 'Color', [1 0.85 0.2], 'LineWidth', 1.6, 'DisplayName', 'HC-SR04 Ray');
    hBot   = plot(ax, 0, 0, 'o', 'MarkerSize', 9, 'MarkerFaceColor', [0 0.95 0.5], 'MarkerEdgeColor', 'w', 'DisplayName', 'AGV Pose');
    hHead  = plot(ax, [0 0.15], [0 0], '-', 'Color', [0 1 0.5], 'LineWidth', 2.5, 'HandleVisibility', 'off');
    lg = legend(ax, [hTrail, hHits, hRay, hBot], 'Location', 'northwest');
    set(lg, 'Color', [0.09 0.11 0.16], 'TextColor', 'w');

    pCtrl = uipanel(fig, 'Title', sprintf('Core Register Bus (%s / %s)', cmdAddr, telemAddr), ...
        'Units', 'normalized', 'Position', [0.64 0.03 0.35 0.94], 'BackgroundColor', pnl, 'ForegroundColor', [0 0.95 0.55], 'FontWeight', 'bold');

    lblState  = mkTxt(pCtrl, [0.04 0.88 0.92 0.05], 'FSM: MANUAL', pnl, [0 0.95 1], 'bold');
    lblPose   = mkTxt(pCtrl, [0.04 0.82 0.92 0.05], 'Pose: (0.00m, 0.00m, 0 deg)', pnl, [1 1 1], 'bold');
    lblSensor = mkTxt(pCtrl, [0.04 0.76 0.92 0.05], 'HW US: -- mm | Line: 000 | Enc: 0/0', pnl, [0.5 0.95 0.6], 'normal');
    lblSim    = mkTxt(pCtrl, [0.04 0.70 0.92 0.05], 'SIM PWM: L=0% R=0% | Trig=0', pnl, [0.85 0.7 1], 'normal');

    lblSpd = mkTxt(pCtrl, [0.04 0.63 0.36 0.04], sprintf('Throttle: %d%%', st.Speed), pnl, [1 0.9 0.3], 'bold');
    hSpdSlider = uicontrol(pCtrl, 'Style', 'slider', 'Units', 'normalized', 'Position', [0.40 0.635 0.56 0.035], ...
        'Min', 15, 'Max', 100, 'Value', st.Speed, 'SliderStep', [0.05 0.15], 'Callback', @onSpeedSlide);

    axVel = axes('Parent', pCtrl, 'Units', 'normalized', 'Position', [0.12 0.44 0.84 0.16], ...
        'Color', [0.04 0.05 0.08], 'XColor', 'w', 'YColor', 'w');
    hVelLine = plot(axVel, NaN, NaN, '-', 'Color', [0 0.95 0.55], 'LineWidth', 1.7);
    ylabel(axVel, 'mm/s', 'Color', 'w', 'FontSize', 8);
    title(axVel, 'Measured Speed (mm/s)', 'Color', [0.8 0.95 1], 'FontSize', 9);
    grid(axVel, 'on');

    uicontrol(pCtrl, 'Style', 'pushbutton', 'Units', 'normalized', 'Position', [0.04 0.33 0.29 0.07], 'String', 'Front LED', 'Callback', @(~,~) togAux(1));
    uicontrol(pCtrl, 'Style', 'pushbutton', 'Units', 'normalized', 'Position', [0.35 0.33 0.29 0.07], 'String', 'FWD (W)', 'BackgroundColor', [0.16 0.55 0.35], 'ForegroundColor', 'w', 'FontWeight', 'bold', 'Callback', @(~,~) btnDrv(1, 1));
    uicontrol(pCtrl, 'Style', 'pushbutton', 'Units', 'normalized', 'Position', [0.66 0.33 0.29 0.07], 'String', 'Rear LED', 'Callback', @(~,~) togAux(2));

    uicontrol(pCtrl, 'Style', 'pushbutton', 'Units', 'normalized', 'Position', [0.04 0.25 0.29 0.07], 'String', 'LEFT (A)', 'BackgroundColor', [0.2 0.4 0.75], 'ForegroundColor', 'w', 'FontWeight', 'bold', 'Callback', @(~,~) btnDrv(-1, 1));
    uicontrol(pCtrl, 'Style', 'pushbutton', 'Units', 'normalized', 'Position', [0.35 0.25 0.29 0.07], 'String', 'STOP (0%)', 'BackgroundColor', [0.3 0.3 0.35], 'ForegroundColor', 'w', 'Callback', @(~,~) btnDrv(0, 0));
    uicontrol(pCtrl, 'Style', 'pushbutton', 'Units', 'normalized', 'Position', [0.66 0.25 0.29 0.07], 'String', 'RIGHT (D)', 'BackgroundColor', [0.2 0.4 0.75], 'ForegroundColor', 'w', 'FontWeight', 'bold', 'Callback', @(~,~) btnDrv(1, -1));

    uicontrol(pCtrl, 'Style', 'pushbutton', 'Units', 'normalized', 'Position', [0.35 0.17 0.29 0.07], 'String', 'REV (S)', 'BackgroundColor', [0.6 0.3 0.2], 'ForegroundColor', 'w', 'FontWeight', 'bold', 'Callback', @(~,~) btnDrv(-1, -1));

    uicontrol(pCtrl, 'Style', 'pushbutton', 'Units', 'normalized', 'Position', [0.04 0.09 0.44 0.06], 'String', 'LINE PID (L)', 'BackgroundColor', [0.18 0.45 0.65], 'ForegroundColor', 'w', 'FontWeight', 'bold', 'Callback', @(~,~) toggleLine());
    uicontrol(pCtrl, 'Style', 'pushbutton', 'Units', 'normalized', 'Position', [0.52 0.09 0.43 0.06], 'String', 'RESET MAP (R)', 'BackgroundColor', [0.25 0.30 0.38], 'ForegroundColor', 'w', 'Callback', @(~,~) resetWorld());
    uicontrol(pCtrl, 'Style', 'pushbutton', 'Units', 'normalized', 'Position', [0.04 0.02 0.91 0.06], 'String', 'EMERGENCY STOP (SPACE)', 'BackgroundColor', [0.80 0.12 0.16], 'ForegroundColor', 'w', 'FontWeight', 'bold', 'Callback', @(~,~) setMode(0));

    chkBumper = uicontrol(pCtrl, 'Style', 'checkbox', 'Units', 'normalized', 'Position', [0.04 0.94 0.30 0.04], 'String', 'Wall Bumper', 'Value', 1, 'BackgroundColor', pnl, 'ForegroundColor', [1 0.65 0.5]);
    chkTrack  = uicontrol(pCtrl, 'Style', 'checkbox', 'Units', 'normalized', 'Position', [0.34 0.94 0.30 0.04], 'String', 'Floor Track', 'Value', 1, 'BackgroundColor', pnl, 'ForegroundColor', [0.8 0.95 1]);
    chkReveal = uicontrol(pCtrl, 'Style', 'checkbox', 'Units', 'normalized', 'Position', [0.65 0.94 0.33 0.04], 'String', 'Reveal Truth (G)', 'Value', double(st.RevealTruth), 'BackgroundColor', pnl, 'ForegroundColor', [1 0.85 0.4], 'Callback', @onToggleReveal);

    drawnow;
    pause(0.05);

    tmr = timer('ExecutionMode', 'fixedSpacing', 'Period', 0.10, 'BusyMode', 'drop', 'TimerFcn', @(~,~) onTick());
    start(tmr);

    function onTick()
        if busyTick || ~st.Running || ~ishandle(fig)
            return;
        end
        busyTick = true;

        try
            updKeys();
            w0 = pack_dut(st.Magic, st.Mode, st.Aux, st.CmdL, st.CmdR);
            st.Magic = 1;

            esp32_cli('on', dutPort, 'write', cmdAddr, w0);
            dutW = esp32_cli('on', dutPort, 'dump', telemAddr, telemWords);
            d = decode_dut(dutW);

            xM = d.x_mm / 1000.0;
            yM = d.y_mm / 1000.0;
            th = d.theta_rad;
            sensX = xM + usOffsetM * cos(th);
            sensY = yM + usOffsetM * sin(th);

            virtDistMm = raycast_room_mm(sensX, sensY, th, world.segments);
            if get(chkBumper, 'Value') == 1 && virtDistMm <= 75 && st.CmdL > 0 && st.CmdR > 0
                st.CmdL = 0; st.CmdR = 0; st.KeyDriving = false;
                esp32_cli('on', dutPort, 'write', cmdAddr, pack_dut(1, 1, st.Aux, 0, 0));
            end

            virtLineMask = 0;
            if get(chkTrack, 'Value') == 1
                virtLineMask = sample_track(xM, yM, th, world.track);
            end

            simCmd = bitor(bitshift(uint32(virtLineMask), 16), uint32(min(max(virtDistMm, 40), 3000)));
            esp32_cli('on', simPort, 'write', cmdAddr, simCmd);
            simW = esp32_cli('on', simPort, 'dump', telemAddr, telemWords);
            simT = decode_sim(simW);

            usM = min(d.us_mm / 1000.0, 2.20);
            isHit = (d.us_mm > 25 && d.us_mm < 2150);
            endX = sensX + usM * cos(th);
            endY = sensY + usM * sin(th);

            [r0, c0] = w2g(sensX, sensY, halfM, resM, nCells);
            [r1, c1] = w2g(endX, endY, halfM, resM, nCells);
            nPts = max(abs(r1 - r0), abs(c1 - c0)) + 1;
            rr = round(linspace(r0, r1, nPts));
            cc = round(linspace(c0, c1, nPts));
            for k = 1:(numel(rr) - 1)
                if rr(k) >= 1 && rr(k) <= nCells && cc(k) >= 1 && cc(k) <= nCells
                    logOdds(rr(k), cc(k)) = max(-4.0, logOdds(rr(k), cc(k)) - 0.35);
                end
            end
            if isHit && r1 >= 1 && r1 <= nCells && c1 >= 1 && c1 <= nCells
                logOdds(r1, c1) = min(4.0, logOdds(r1, c1) + 0.85);
                st.Hits(end + 1, :) = [endX, endY];
                if size(st.Hits, 1) > 600, st.Hits(1, :) = []; end
            end

            st.PoseTrail(end + 1, :) = [xM, yM];
            if size(st.PoseTrail, 1) > 500, st.PoseTrail(1, :) = []; end
            tNow = toc(st.T0);
            st.VelHist(end + 1, :) = [tNow, d.vel_mms];
            st.VelHist = st.VelHist(st.VelHist(:, 1) >= tNow - 25.0, :);

            set(hGrid, 'CData', logodds_to_prob(logOdds));
            set(hTrail, 'XData', st.PoseTrail(:, 1), 'YData', st.PoseTrail(:, 2));
            set(hBot, 'XData', xM, 'YData', yM);
            set(hHead, 'XData', [xM, xM + 0.18 * cos(th)], 'YData', [yM, yM + 0.18 * sin(th)]);
            set(hRay, 'XData', [sensX, endX], 'YData', [sensY, endY]);
            if ~isempty(st.Hits), set(hHits, 'XData', st.Hits(:, 1), 'YData', st.Hits(:, 2)); end
            set(hVelLine, 'XData', st.VelHist(:, 1), 'YData', st.VelHist(:, 2));
            xlim(axVel, [max(0, tNow - 25), max(25, tNow)]);

            fsmNames = {'E-STOP', 'MANUAL', 'LINE_PID'};
            fsmLbl = fsmNames{min(d.fsm + 1, numel(fsmNames))};
            set(lblState, 'String', sprintf('FSM: %s | Cmd=(%d%%, %d%%) | Limit=%d%%', fsmLbl, st.CmdL, st.CmdR, st.Speed));
            set(lblPose, 'String', sprintf('Pose: (%.2fm, %.2fm, %.0f deg) | Vel: %d mm/s', xM, yM, rad2deg(th), d.vel_mms));
            set(lblSensor, 'String', sprintf('HW US: %d mm | Line: %s | Enc: %d/%d', d.us_mm, dec2bin(d.lineMask, 3), d.encL, d.encR));
            set(lblSim, 'String', sprintf('SIM PWM: L=%d%%(%dHz) R=%d%%(%dHz) | Trig=%d', simT.pwmL, simT.freqL, simT.pwmR, simT.freqR, simT.trigCount));
            st.ErrCount = 0;
            drawnow limitrate;
        catch ME
            st.ErrCount = st.ErrCount + 1;
            wb_log('warn', 'TWIN', 'TICK_ERR', sprintf('(%d/5) %s', st.ErrCount, ME.message));
            if st.ErrCount >= 5
                st.Running = false;
                stop(tmr);
                wb_log('error', 'TWIN', 'LINK_LOST', 'Stopped after 5 consecutive tick errors');
                if ishandle(lblState)
                    set(lblState, 'String', 'LINK LOST - Check USB & reopen', 'ForegroundColor', [1 0.3 0.3]);
                end
            end
        end

        busyTick = false;
    end

    function onSpeedSlide(src, ~)
        st.Speed = round(get(src, 'Value'));
        set(lblSpd, 'String', sprintf('Throttle: %d%%', st.Speed));
    end

    function onToggleReveal(src, ~)
        st.RevealTruth = (get(src, 'Value') == 1);
        vis = 'off';
        if st.RevealTruth, vis = 'on'; end
        set(hTruthGrp, 'Visible', vis);
    end

    function onKeyDown(~, e)
        k = lower(char(e.Key));
        if strcmp(k, 'space'), setMode(0); return;
        elseif strcmp(k, 'l'), toggleLine(); return;
        elseif strcmp(k, 'r'), resetWorld(); return;
        elseif strcmp(k, 'g'), set(chkReveal, 'Value', double(~st.RevealTruth)); onToggleReveal(chkReveal, []); return;
        elseif any(strcmp(k, {'equal', 'add', 'rightbracket'})), adjSpeed(10); return;
        elseif any(strcmp(k, {'hyphen', 'subtract', 'leftbracket'})), adjSpeed(-10); return;
        end
        st.Keys.(matlab.lang.makeValidName(['k_' k])) = true;
    end

    function onKeyUp(~, e)
        st.Keys.(matlab.lang.makeValidName(['k_' lower(char(e.Key))])) = false;
    end

    function adjSpeed(delta)
        st.Speed = min(100, max(15, st.Speed + delta));
        set(hSpdSlider, 'Value', st.Speed);
        set(lblSpd, 'String', sprintf('Throttle: %d%%', st.Speed));
    end

    function updKeys()
        if st.Mode ~= 1, return; end
        vy = double(keyOn('w') || keyOn('uparrow')) - double(keyOn('s') || keyOn('downarrow'));
        vx = double(keyOn('d') || keyOn('rightarrow')) - double(keyOn('a') || keyOn('leftarrow'));
        if vy ~= 0 || vx ~= 0
            st.KeyDriving = true;
            st.CmdL = max(-100, min(100, round((vy + 0.75 * vx) * st.Speed)));
            st.CmdR = max(-100, min(100, round((vy - 0.75 * vx) * st.Speed)));
        elseif st.KeyDriving
            st.KeyDriving = false; st.CmdL = 0; st.CmdR = 0;
        end
    end

    function ok = keyOn(k)
        fn = matlab.lang.makeValidName(['k_' k]);
        ok = isfield(st.Keys, fn) && st.Keys.(fn);
    end

    function btnDrv(l, r)
        st.KeyDriving = false; st.Mode = 1;
        st.CmdL = round(l * st.Speed); st.CmdR = round(r * st.Speed);
    end

    function toggleLine()
        if st.Mode == 2, setMode(1); else, setMode(2); end
    end

    function setMode(m)
        st.KeyDriving = false; st.Mode = m;
        if m ~= 1, st.CmdL = 0; st.CmdR = 0; end
    end

    function togAux(b), st.Aux = bitxor(st.Aux, b); end

    function resetWorld()
        st.Magic = 15; logOdds(:) = 0;
        st.PoseTrail = zeros(0, 2); st.Hits = zeros(0, 2);
        set(hHits, 'XData', NaN, 'YData', NaN);
        set(hTrail, 'XData', NaN, 'YData', NaN);
    end

    function onClose(~, ~)
        st.Running = false;
        stop(tmr);
        delete(tmr);
        if esp32_io('state', dutPort).Connected
            esp32_cli('on', dutPort, 'write', cmdAddr, pack_dut(1, 0, 0, 0, 0));
            esp32_cli('disconnect', dutPort);
        end
        if esp32_io('state', simPort).Connected
            esp32_cli('disconnect', simPort);
        end
        delete(fig);
    end
end

function w = pack_dut(magic, mode, aux, l, r)
    w = bitor( ...
        bitor(bitshift(uint32(magic), 28), bitshift(uint32(mode), 24)), ...
        bitor(bitshift(bitand(uint32(aux), 255), 16), bitor(bitshift(uint32(bitand(int32(l), 255)), 8), uint32(bitand(int32(r), 255)))));
end

function d = decode_dut(w)
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

function s = decode_sim(w)
    s.pwmL      = i16(bitshift(w(1), -16));
    s.pwmR      = i16(bitand(w(1), 65535));
    s.freqL     = double(bitshift(w(2), -16));
    s.freqR     = double(bitand(w(2), 65535));
    s.ledMask   = double(bitand(bitshift(w(3), -16), 15));
    s.trigCount = double(bitand(w(3), 65535));
end

function [world, logOdds, axCoords, nCells, halfM, resM] = load_world_map(mapFile)
    m = jsondecode(fileread(mapFile));
    segs = zeros(0, 4);
    for g = 1:numel(m.walls)
        sMat = m.walls(g).segments;
        if iscell(sMat), sMat = cell2mat(sMat); end
        segs = [segs; sMat]; %#ok<AGROW>
    end
    world = struct('name', m.name, 'segments', segs, 'track', m.track);
    resM = m.resolutionM;
    halfM = max(abs(m.boundsM));
    nCells = round((2 * halfM) / resM);
    logOdds = zeros(nCells, nCells);
    axCoords = linspace(-halfM, halfM, nCells);
end

function draw_truth_overlay(grp, world)
    if isfield(world.track, 'type') && strcmpi(world.track.type, 'oval')
        thTrk = linspace(0, 2 * pi, 120);
        plot(grp, world.track.rx * cos(thTrk), world.track.ry * sin(thTrk), ':', 'Color', [0.2 0.85 0.45], 'LineWidth', 1.4);
    end
    for w = 1:size(world.segments, 1)
        plot(grp, [world.segments(w, 1) world.segments(w, 3)], [world.segments(w, 2) world.segments(w, 4)], '--', 'Color', [0 0.85 1 0.6], 'LineWidth', 1.5);
    end
end

function distMm = raycast_room_mm(ox, oy, th, segs)
    dx = cos(th); dy = sin(th); bestT = 2.50;
    for k = 1:size(segs, 1)
        x1 = segs(k, 1); y1 = segs(k, 2); sx = segs(k, 3) - x1; sy = segs(k, 4) - y1;
        den = dx * sy - dy * sx;
        if abs(den) < 1e-6, continue; end
        t = ((x1 - ox) * sy - (y1 - oy) * sx) / den;
        u = ((x1 - ox) * dy - (y1 - oy) * dx) / den;
        if t > 0.02 && t < bestT && u >= 0.0 && u <= 1.0, bestT = t; end
    end
    distMm = round(bestT * 1000.0);
end

function msk = sample_track(x, y, th, trk)
    msk = 0;
    if ~isfield(trk, 'type') || ~strcmpi(trk.type, 'oval'), return; end
    offsets = [0.022, 0.0, -0.022];
    bits = [4, 2, 1];
    for k = 1:3
        px = x + 0.070 * cos(th) - offsets(k) * sin(th);
        py = y + 0.070 * sin(th) + offsets(k) * cos(th);
        rVal = sqrt((px / trk.rx)^2 + (py / trk.ry)^2);
        if abs(rVal - 1.0) < 0.06, msk = bitor(msk, bits(k)); end
    end
end

function [r, c] = w2g(x, y, hM, res, n)
    c = min(max(round((x + hM) / res) + 1, 1), n);
    r = min(max(round((y + hM) / res) + 1, 1), n);
end

function p = logodds_to_prob(lo)
    p = 1.0 - (1.0 ./ (1.0 + exp(lo)));
end

function h = mkTxt(p, pos, str, bg, fg, fw)
    h = uicontrol(p, 'Style', 'text', 'Units', 'normalized', 'Position', pos, 'String', str, ...
        'BackgroundColor', bg, 'ForegroundColor', fg, 'FontWeight', fw, 'HorizontalAlignment', 'left', 'FontSize', 10);
end

function v = i16(u)
    v = double(int32(u));
    if v >= 32768, v = v - 65536; end
end