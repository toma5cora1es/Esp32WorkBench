function mapOut = classroom_mapper(masterPort, boardPath, roomCfgPath)
%CLASSROOM_MAPPER Real-world wireless classroom AGV mapper built directly on core/esp32_cli.

    if nargin < 2 || isempty(boardPath)
        boardPath = 'experiments/classroom_agv/classroom_agv_board.json';
    end
    if nargin < 3 || isempty(roomCfgPath)
        roomCfgPath = 'experiments/classroom_agv/classroom_config.json';
    end

    prev = findall(0, 'Type', 'figure', 'Tag', 'CLASSROOM_MAPPER');
    if ~isempty(prev)
        close(prev);
        drawnow;
    end

    board = wb_config('board', boardPath);
    room = jsondecode(fileread(wb_paths('resolve', roomCfgPath)));

    cmdAddr = board.registers.commandWordAddr;
    telemAddr = board.registers.telemetryBaseAddr;
    nWords = board.registers.telemetryWordCount;
    usOffsetM = board.kinematics.ultrasonicForwardOffsetMm / 1000.0;

    esp32_cli('connect', masterPort, room.serialBaud);
    esp32_cli('write', cmdAddr, pack_cmd(15, 1, 0, 0, 0));

    resM = room.gridResolutionM;
    xVec = room.boundsM.xMin:resM:room.boundsM.xMax;
    yVec = room.boundsM.yMin:resM:room.boundsM.yMax;
    logOdds = zeros(numel(yVec), numel(xVec));

    st = struct('Running', true, 'Mode', 1, 'Aux', 0, 'Speed', room.safety.defaultThrottlePct, ...
                'CmdL', 0, 'CmdR', 0, 'Magic', 1, 'KeyDriving', false, 'Keys', struct(), ...
                'PrevTheta', 0.0, 'PrevTickT', tic, 'PoseTrail', zeros(0, 2), 'Hits', zeros(0, 2), 'ErrCount', 0);
    busyTick = false;

    bg = [0.08 0.10 0.14];
    pnl = [0.06 0.08 0.11];
    fig = figure('Name', sprintf('Classroom Real AGV Mapper (%s)', upper(masterPort)), ...
        'NumberTitle', 'off', 'MenuBar', 'none', 'ToolBar', 'none', 'Color', bg, 'Position', [40 40 1320 740], ...
        'Tag', 'CLASSROOM_MAPPER', 'WindowKeyPressFcn', @onKeyDown, 'WindowKeyReleaseFcn', @onKeyUp, 'CloseRequestFcn', @onClose);

    pMap = uipanel(fig, 'Title', room.roomName, 'Units', 'normalized', 'Position', [0.01 0.03 0.64 0.94], ...
        'BackgroundColor', pnl, 'ForegroundColor', [0 0.95 1], 'FontWeight', 'bold');
    ax = axes('Parent', pMap, 'Units', 'normalized', 'Position', [0.08 0.09 0.88 0.86], ...
        'Color', [0.03 0.04 0.07], 'XColor', 'w', 'YColor', 'w');
    hold(ax, 'on'); axis(ax, 'equal'); xlim(ax, [room.boundsM.xMin room.boundsM.xMax]); ylim(ax, [room.boundsM.yMin room.boundsM.yMax]); grid(ax, 'on');
    xlabel(ax, 'Classroom X (m) — [WASD/Arrows=Drive | Space=E-STOP | R=Reset]', 'Color', 'w');
    ylabel(ax, 'Classroom Y (m)', 'Color', 'w');

    hGrid = imagesc(ax, xVec, yVec, 1.0 - (1.0 ./ (1.0 + exp(logOdds))));
    set(ax, 'YDir', 'normal');
    cmap = [linspace(0.10, 0.03, 128)' linspace(0.24, 0.04, 128)' linspace(0.30, 0.07, 128)'; ...
            linspace(0.03, 0.98, 128)' linspace(0.04, 0.92, 128)' linspace(0.07, 0.55, 128)'];
    colormap(ax, cmap); caxis(ax, [0 1]);

    hTrail = plot(ax, NaN, NaN, '-', 'Color', [0.15 0.72 1.0], 'LineWidth', 2.0);
    hHits  = scatter(ax, NaN, NaN, 24, [1 0.25 0.25], 'filled');
    hRay   = plot(ax, NaN, NaN, '--', 'Color', [1 0.85 0.2], 'LineWidth', 1.5);
    hBot   = plot(ax, 0, 0, 'o', 'MarkerSize', 9, 'MarkerFaceColor', [0 0.95 0.5], 'MarkerEdgeColor', 'w');

    pCtrl = uipanel(fig, 'Title', 'Wireless Telemetry & Safety', 'Units', 'normalized', ...
        'Position', [0.66 0.03 0.33 0.94], 'BackgroundColor', pnl, 'ForegroundColor', [0 0.95 0.55], 'FontWeight', 'bold');

    lblLink   = mkTxt(pCtrl, [0.04 0.88 0.92 0.05], 'ESP-NOW: -- ms RTT | Age: -- ms', pnl, [1 0.9 0.3], 'bold');
    lblState  = mkTxt(pCtrl, [0.04 0.82 0.92 0.05], 'FSM: MANUAL', pnl, [0 0.95 1], 'bold');
    lblPose   = mkTxt(pCtrl, [0.04 0.76 0.92 0.05], 'Pose: (0.00m, 0.00m, 0 deg) | Vel: 0 mm/s', pnl, [1 1 1], 'bold');
    lblSensor = mkTxt(pCtrl, [0.04 0.70 0.92 0.05], 'US: -- mm | Line: 000 | Enc: 0/0', pnl, [0.5 0.95 0.6], 'normal');

    lblSpd = mkTxt(pCtrl, [0.04 0.62 0.38 0.04], sprintf('Speed Cap: %d%%', st.Speed), pnl, [1 0.9 0.3], 'bold');
    uicontrol(pCtrl, 'Style', 'slider', 'Units', 'normalized', 'Position', [0.44 0.625 0.52 0.035], ...
        'Min', 15, 'Max', 100, 'Value', st.Speed, 'Callback', @(s,~) setSpeed(round(get(s, 'Value'))));

    uicontrol(pCtrl, 'Style', 'pushbutton', 'Units', 'normalized', 'Position', [0.04 0.48 0.44 0.07], 'String', 'Front LED', 'Callback', @(~,~) togAux(1));
    uicontrol(pCtrl, 'Style', 'pushbutton', 'Units', 'normalized', 'Position', [0.52 0.48 0.44 0.07], 'String', 'Rear LED', 'Callback', @(~,~) togAux(2));
    uicontrol(pCtrl, 'Style', 'pushbutton', 'Units', 'normalized', 'Position', [0.34 0.38 0.32 0.08], 'String', 'FWD (W)', 'BackgroundColor', [0.16 0.55 0.35], 'ForegroundColor', 'w', 'Callback', @(~,~) btnDrv(1, 1));
    uicontrol(pCtrl, 'Style', 'pushbutton', 'Units', 'normalized', 'Position', [0.04 0.29 0.28 0.08], 'String', 'LEFT (A)', 'BackgroundColor', [0.2 0.4 0.75], 'ForegroundColor', 'w', 'Callback', @(~,~) btnDrv(-1, 1));
    uicontrol(pCtrl, 'Style', 'pushbutton', 'Units', 'normalized', 'Position', [0.34 0.29 0.32 0.08], 'String', 'STOP', 'BackgroundColor', [0.3 0.3 0.35], 'ForegroundColor', 'w', 'Callback', @(~,~) btnDrv(0, 0));
    uicontrol(pCtrl, 'Style', 'pushbutton', 'Units', 'normalized', 'Position', [0.68 0.29 0.28 0.08], 'String', 'RIGHT (D)', 'BackgroundColor', [0.2 0.4 0.75], 'ForegroundColor', 'w', 'Callback', @(~,~) btnDrv(1, -1));
    uicontrol(pCtrl, 'Style', 'pushbutton', 'Units', 'normalized', 'Position', [0.34 0.20 0.32 0.08], 'String', 'REV (S)', 'BackgroundColor', [0.6 0.3 0.2], 'ForegroundColor', 'w', 'Callback', @(~,~) btnDrv(-1, -1));
    uicontrol(pCtrl, 'Style', 'pushbutton', 'Units', 'normalized', 'Position', [0.04 0.11 0.92 0.06], 'String', 'RESET POSE & MAP (R)', 'BackgroundColor', [0.25 0.30 0.38], 'ForegroundColor', 'w', 'Callback', @(~,~) resetMap());
    uicontrol(pCtrl, 'Style', 'pushbutton', 'Units', 'normalized', 'Position', [0.04 0.03 0.92 0.07], 'String', 'EMERGENCY STOP (SPACE)', 'BackgroundColor', [0.80 0.12 0.16], 'ForegroundColor', 'w', 'FontWeight', 'bold', 'Callback', @(~,~) setMode(0));

    drawnow;
    pause(0.05);

    tmr = timer('ExecutionMode', 'fixedSpacing', 'Period', 0.06, 'BusyMode', 'drop', 'TimerFcn', @(~,~) onTick());
    start(tmr);

    if nargout > 0
        waitfor(fig);
        mapOut = struct('xVec', xVec, 'yVec', yVec, 'probGrid', 1.0 - (1.0 ./ (1.0 + exp(logOdds))), 'poseTrail', st.PoseTrail, 'hits', st.Hits);
    end

    function onTick()
        if busyTick || ~st.Running || ~ishandle(fig), return; end
        busyTick = true;

        try
            updKeys();
            if st.Magic ~= 15
                st.Magic = 3 - st.Magic;
            end
            w0 = pack_cmd(st.Magic, st.Mode, st.Aux, st.CmdL, st.CmdR);
            if st.Magic == 15
                st.Magic = 1;
            end

            esp32_cli('write', cmdAddr, w0);
            w = esp32_cli('dump', telemAddr, nWords);
            rttUs = double(esp32_cli('read', board.registers.linkRttUsAddr));
            ageMs = double(esp32_cli('read', board.registers.linkAgeMsAddr));

            d = decode_telem(w);
            dt = max(toc(st.PrevTickT), 0.02);
            st.PrevTickT = tic;
            omegaDegSec = abs(rad2deg(atan2(sin(d.theta_rad - st.PrevTheta), cos(d.theta_rad - st.PrevTheta)))) / dt;
            st.PrevTheta = d.theta_rad;

            xM = d.x_mm / 1000.0;
            yM = d.y_mm / 1000.0;
            th = d.theta_rad;
            sensX = xM + usOffsetM * cos(th);
            sensY = yM + usOffsetM * sin(th);

            usM = min(d.us_mm / 1000.0, room.sonar.maxReliableMm / 1000.0);
            endX = sensX + usM * cos(th);
            endY = sensY + usM * sin(th);

            if ageMs <= room.safety.maxLinkAgeMs && omegaDegSec <= room.sonar.maxOmegaHitDegSec
                [r0, c0] = w2g(sensX, sensY, xVec(1), yVec(1), resM, numel(xVec), numel(yVec));
                [r1, c1] = w2g(endX, endY, xVec(1), yVec(1), resM, numel(xVec), numel(yVec));
                nPts = max(abs(r1 - r0), abs(c1 - c0)) + 1;
                rr = round(linspace(r0, r1, nPts));
                cc = round(linspace(c0, c1, nPts));
                for k = 1:(numel(rr) - 1)
                    logOdds(rr(k), cc(k)) = max(-4.0, logOdds(rr(k), cc(k)) + room.sonar.logOddsFree);
                end
                if d.us_mm >= room.sonar.minValidMm && d.us_mm <= room.sonar.maxReliableMm
                    logOdds(r1, c1) = min(4.0, logOdds(r1, c1) + room.sonar.logOddsHit);
                    st.Hits(end + 1, :) = [endX, endY];
                end
            end

            st.PoseTrail(end + 1, :) = [xM, yM];
            set(hGrid, 'CData', 1.0 - (1.0 ./ (1.0 + exp(logOdds))));
            set(hTrail, 'XData', st.PoseTrail(:, 1), 'YData', st.PoseTrail(:, 2));
            set(hBot, 'XData', xM, 'YData', yM);
            set(hRay, 'XData', [sensX, endX], 'YData', [sensY, endY]);
            if ~isempty(st.Hits), set(hHits, 'XData', st.Hits(:, 1), 'YData', st.Hits(:, 2)); end

            fsmNames = {'E-STOP', 'MANUAL', 'LINE', 'SLAM', 'AUTO-BRAKE'};
            set(lblLink, 'String', sprintf('ESP-NOW: %.1f ms RTT | Age: %d ms', rttUs / 1000.0, ageMs));
            set(lblState, 'String', sprintf('FSM: %s | Cmd=(%d%%, %d%%)', fsmNames{min(d.fsm + 1, 5)}, st.CmdL, st.CmdR));
            set(lblPose, 'String', sprintf('Pose: (%.2fm, %.2fm, %.0f deg) | Vel: %d mm/s', xM, yM, rad2deg(th), d.vel_mms));
            set(lblSensor, 'String', sprintf('US: %d mm | Line: %s | Enc: %d/%d', d.us_mm, dec2bin(d.lineMask, 3), d.encL, d.encR));
            st.ErrCount = 0;
            drawnow limitrate;
        catch ME
            st.ErrCount = st.ErrCount + 1;
            wb_log('warn', 'MAPPER', 'TICK_ERR', ME.message);
            if st.ErrCount >= 5
                st.Running = false;
                stop(tmr);
            end
        end
        busyTick = false;
    end

    function setSpeed(v), st.Speed = v; set(lblSpd, 'String', sprintf('Speed Cap: %d%%', v)); end
    function onKeyDown(~, e)
        k = lower(char(e.Key));
        if strcmp(k, 'space'), setMode(0); return;
        elseif strcmp(k, 'r'), resetMap(); return;
        end
        st.Keys.(matlab.lang.makeValidName(['k_' k])) = true;
    end
    function onKeyUp(~, e), st.Keys.(matlab.lang.makeValidName(['k_' lower(char(e.Key))])) = false; end
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
    function ok = keyOn(k), fn = matlab.lang.makeValidName(['k_' k]); ok = isfield(st.Keys, fn) && st.Keys.(fn); end
    function btnDrv(l, r), st.KeyDriving = false; st.Mode = 1; st.CmdL = round(l * st.Speed); st.CmdR = round(r * st.Speed); end
    function setMode(m), st.KeyDriving = false; st.Mode = m; if m ~= 1, st.CmdL = 0; st.CmdR = 0; end; end
    function togAux(b), st.Aux = bitxor(st.Aux, b); end
    function resetMap(), st.Magic = 15; logOdds(:) = 0; st.PoseTrail = zeros(0, 2); st.Hits = zeros(0, 2); end
    function onClose(~, ~)
        st.Running = false;
        stop(tmr);
        delete(tmr);
        if esp32_io('state', masterPort).Connected
            esp32_cli('write', cmdAddr, pack_cmd(1, 0, 0, 0, 0));
            esp32_cli('disconnect', masterPort);
        end
        delete(fig);
    end
end

function [r, c] = w2g(x, y, x0, y0, res, nx, ny)
    c = min(max(round((x - x0) / res) + 1, 1), nx);
    r = min(max(round((y - y0) / res) + 1, 1), ny);
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

function h = mkTxt(p, pos, str, bg, fg, fw)
    h = uicontrol(p, 'Style', 'text', 'Units', 'normalized', 'Position', pos, 'String', str, ...
        'BackgroundColor', bg, 'ForegroundColor', fg, 'FontWeight', fw, 'HorizontalAlignment', 'left', 'FontSize', 10);
end

function v = i16(u)
    v = double(int32(u));
    if v >= 32768, v = v - 65536; end
end