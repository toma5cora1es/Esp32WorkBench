function report = esp32_flash(action, sourceKind, source, varargin)
%ESP32_FLASH Explicit ESP32 compiler and USB uploader.

    cfg = wb_config('workbench');
    act = lower(strtrim(char(action)));

    if ~ismember(act, {'compile', 'flash'})
        wb_log('error', 'FLASH', 'BAD_ACTION', sprintf('Action must be compile or flash (got "%s")', act));
        error('ESP32:BadFlashAction', 'Action must be compile or flash.');
    end

    if strcmp(act, 'flash')
        if isempty(varargin) || isempty(varargin{1})
            wb_log('error', 'FLASH', 'MISSING_PORT', 'Explicit serial port is required for flash (e.g. COM9)');
            error('ESP32:MissingPort', 'Specify serial port explicitly for flash.');
        end
        port = upper(strtrim(char(varargin{1})));
        optArgs = varargin(2:end);

        s = esp32_io('state', port);
        if s.Connected
            wb_log('error', 'FLASH', 'PORT_IN_USE', sprintf('%s is connected in esp32_io; disconnect before flashing', port));
            error('ESP32:PortInUse', '%s is open in esp32_io. Call esp32_cli(''disconnect'', ''%s'') first.', port, port);
        end
    else
        port = '';
        optArgs = varargin;
    end

    opts = parse_flash_options(optArgs{:});
    if isempty(opts.partitions)
        opts.partitions = char(cfg.toolchain.partitionScheme);
    end

    [sketchCode, srcLabel, companionDir] = load_explicit_source(sourceKind, source);
    buildId = sprintf('BLD_%s', char(datetime('now', 'Format', 'yyyyMMdd_HHmmss')));

    if isempty(opts.outDir)
        buildRoot = fullfile(wb_paths('build'), buildId);
    else
        buildRoot = wb_paths('resolve', opts.outDir);
    end

    sketchDir = fullfile(buildRoot, 'sketch');
    outBinDir = fullfile(buildRoot, 'bin');

    if isfolder(buildRoot)
        rmdir(buildRoot, 's');
    end
    mkdir(sketchDir);
    mkdir(outBinDir);

    if ~isempty(companionDir)
        copy_companions(companionDir, sketchDir);
    end

    outIno = fullfile(sketchDir, 'sketch.ino');
    write_utf8(outIno, sketchCode);

    if ~isempty(opts.board)
        boardCfg = wb_config('board', opts.board);
        boardHdr = fullfile(sketchDir, 'board_pins.h');
        write_utf8(boardHdr, build_board_header(boardCfg));
        wb_log('normal', 'FLASH', 'BOARD_HDR', sprintf('Emitted %s (%s)', wb_paths('rel', boardHdr), boardCfg.boardName));
    end

    wb_log('normal', 'FLASH', 'SOURCE', sprintf('%s -> %s', srcLabel, wb_paths('rel', outIno)));

    acli = find_acli(cfg);
    libDir = wb_paths('lib');
    cmdCompile = sprintf('"%s" compile --fqbn %s --libraries "%s" --build-property "build.partitions=%s" --build-property "upload.maximum_size=%d" --output-dir "%s" "%s"', ...
        acli, cfg.toolchain.fqbn, libDir, opts.partitions, cfg.toolchain.maxAppBytes, outBinDir, sketchDir);

    wb_log('normal', 'FLASH', 'CMD', cmdCompile);
    [st, cout] = system(cmdCompile);
    if st ~= 0
        wb_log('error', 'FLASH', 'COMPILE_FAIL', sprintf('arduino-cli compile exited with %d', st));
        fprintf(2, '%s\n', cout);
        error('ESP32:CompileFail', 'arduino-cli compile failed.');
    end

    [flashUsed, flashMax, flashPct, ramUsed, ramMax, ramPct] = parse_size_summary(cout);
    artifacts = collect_artifacts(outBinDir);

    if ~isnan(flashPct)
        wb_log('normal', 'FLASH', 'SIZE', sprintf('Flash: %d / %d B (%.1f%%) | RAM: %d / %d B (%.1f%%)', ...
            flashUsed, flashMax, flashPct, ramUsed, ramMax, ramPct));
        if flashPct >= 80.0
            wb_log('warn', 'FLASH', 'SPACE_HIGH', sprintf('Flash usage is %.1f%% (%d / %d B)', flashPct, flashUsed, flashMax));
        end
    else
        wb_log('warn', 'FLASH', 'SIZE_PARSE', 'Could not parse Flash/RAM usage from arduino-cli output');
    end

    for k = 1:numel(artifacts)
        wb_log('normal', 'FLASH', 'ARTIFACT', sprintf('%s (%d B)', artifacts(k).name, artifacts(k).bytes));
    end

    binFile = fullfile(outBinDir, 'sketch.ino.bin');
    uploaded = false;

    if strcmp(act, 'flash')
        stale = serialportfind('Port', port);
        if ~isempty(stale)
            wb_log('warn', 'FLASH', 'STALE_HANDLE', sprintf('Closing orphaned MATLAB handle on %s', port));
            delete(stale);
        end

        cmdUpload = sprintf('"%s" upload -p %s --fqbn %s --input-dir "%s"', acli, port, cfg.toolchain.fqbn, outBinDir);
        wb_log('normal', 'FLASH', 'UPLOAD', cmdUpload);
        [st, uout] = system(cmdUpload);
        if st ~= 0
            wb_log('error', 'FLASH', 'UPLOAD_FAIL', sprintf('arduino-cli upload failed on %s (exit %d)', port, st));
            fprintf(2, '%s\n', uout);
            error('ESP32:UploadFail', 'USB upload failed on %s.', port);
        end

        pause(1.2);
        uploaded = true;
        wb_log('normal', 'FLASH', 'COMPLETE', sprintf('Flashed %s via USB (%s)', buildId, port));
    end

    report = struct( ...
        'buildId',    buildId, ...
        'action',     act, ...
        'source',     srcLabel, ...
        'board',      char(opts.board), ...
        'partitions', opts.partitions, ...
        'sketchDir',  wb_paths('rel', sketchDir), ...
        'binDir',     wb_paths('rel', outBinDir), ...
        'binFile',    wb_paths('rel', binFile), ...
        'flashUsed',  flashUsed, ...
        'flashMax',   flashMax, ...
        'flashPct',   flashPct, ...
        'ramUsed',    ramUsed, ...
        'ramMax',     ramMax, ...
        'ramPct',     ramPct, ...
        'artifacts',  artifacts, ...
        'uploaded',   uploaded);

    manifestPath = fullfile(buildRoot, 'build_manifest.json');
    write_utf8(manifestPath, jsonencode(report, 'PrettyPrint', true));
    wb_log('normal', 'FLASH', 'MANIFEST', wb_paths('rel', manifestPath));
end

function [code, label, companionDir] = load_explicit_source(sourceKind, source)
    kind = lower(strtrim(char(sourceKind)));
    companionDir = '';

    switch kind
        case 'inline'
            if isstring(source) || iscell(source)
                lines = string(source);
                code = char(strjoin(lines(:), newline));
                label = sprintf('Inline (%d lines)', numel(lines));
            else
                code = char(source);
                label = sprintf('Inline (%d bytes)', numel(code));
            end

        case 'file'
            sketchPath = wb_paths('sketch', source);
            code = fileread(sketchPath);
            label = wb_paths('rel', sketchPath);

            [parentDir, fileBase] = fileparts(sketchPath);
            [~, dirName] = fileparts(parentDir);
            if strcmpi(fileBase, dirName)
                companionDir = parentDir;
            end

        otherwise
            wb_log('error', 'FLASH', 'BAD_KIND', sprintf('Source kind must be file or inline (got "%s")', kind));
            error('ESP32:BadSourceKind', 'Source kind must be ''file'' or ''inline''.');
    end
end

function copy_companions(srcDir, dstDir)
    exts = {'*.h', '*.hpp', '*.cpp', '*.c'};
    for e = 1:numel(exts)
        files = dir(fullfile(srcDir, exts{e}));
        for k = 1:numel(files)
            copyfile(fullfile(files(k).folder, files(k).name), fullfile(dstDir, files(k).name));
            wb_log('normal', 'FLASH', 'COMPANION', files(k).name);
        end
    end
end

function opts = parse_flash_options(varargin)
    opts = struct('board', '', 'outDir', '', 'partitions', '');
    if mod(numel(varargin), 2) ~= 0
        wb_log('error', 'FLASH', 'OPTIONS', 'Flash options must be name-value pairs');
        error('ESP32:BadFlashOptions', 'Flash options must be name-value pairs.');
    end

    for k = 1:2:numel(varargin)
        key = lower(strtrim(char(varargin{k})));
        val = varargin{k + 1};
        switch key
            case 'board'
                opts.board = strtrim(char(val));
            case 'outdir'
                opts.outDir = strtrim(char(val));
            case 'partitions'
                opts.partitions = strtrim(char(val));
            otherwise
                wb_log('error', 'FLASH', 'BAD_OPTION', sprintf('Unknown flash option "%s"', key));
                error('ESP32:BadFlashOption', 'Unknown flash option "%s".', key);
        end
    end
end

function hdr = build_board_header(boardCfg)
    fns = fieldnames(boardCfg.macros);
    lines = strings(numel(fns) + 2, 1);
    lines(1) = '#pragma once';
    lines(2) = sprintf('// Generated from board profile: %s', boardCfg.boardName);
    for k = 1:numel(fns)
        v = boardCfg.macros.(fns{k});
        if isnumeric(v)
            vs = num2str(v);
        elseif islogical(v)
            vs = num2str(double(v));
        else
            vs = char(v);
        end
        lines(k + 2) = sprintf('#define %s %s', fns{k}, vs);
    end
    hdr = char(strjoin(lines, newline));
end

function [fUsed, fMax, fPct, rUsed, rMax, rPct] = parse_size_summary(cout)
    fUsed = NaN; fMax = NaN; fPct = NaN;
    rUsed = NaN; rMax = NaN; rPct = NaN;

    mF = regexp(cout, 'Sketch uses\s+(\d+)\s+bytes.*?Maximum is\s+(\d+)\s+bytes', 'tokens', 'once');
    if ~isempty(mF)
        fUsed = str2double(mF{1});
        fMax = str2double(mF{2});
        fPct = 100.0 * fUsed / fMax;
    end

    mR = regexp(cout, 'Global variables use\s+(\d+)\s+bytes.*?Maximum is\s+(\d+)\s+bytes', 'tokens', 'once');
    if ~isempty(mR)
        rUsed = str2double(mR{1});
        rMax = str2double(mR{2});
        rPct = 100.0 * rUsed / rMax;
    end
end

function artifacts = collect_artifacts(binDir)
    d = dir(binDir);
    d = d(~[d.isdir]);
    artifacts = struct('name', {}, 'bytes', {});
    for k = 1:numel(d)
        artifacts(k).name = d(k).name;
        artifacts(k).bytes = d(k).bytes;
    end
end

function acli = find_acli(cfg)
    checkCmd = 'which arduino-cli >/dev/null 2>&1';
    if ispc
        checkCmd = 'where arduino-cli >nul 2>&1';
    end

    [st, ~] = system(checkCmd);
    if st == 0
        acli = char(cfg.toolchain.arduinoCliBinary);
        return;
    end

    for k = 1:numel(cfg.toolchain.arduinoCliSearchPaths)
        c = char(cfg.toolchain.arduinoCliSearchPaths{k});
        toks = regexp(c, '%([^%]+)%', 'tokens');
        for t = 1:numel(toks)
            v = getenv(toks{t}{1});
            if ~isempty(v)
                c = strrep(c, ['%' toks{t}{1} '%'], v);
            end
        end
        if isfile(c)
            acli = c;
            return;
        end
    end

    wb_log('error', 'FLASH', 'NO_ARDUINO_CLI', 'arduino-cli binary not found in PATH or config search paths');
    error('ESP32:NoArduinoCli', 'arduino-cli not found.');
end

function write_utf8(p, txt)
    fid = fopen(p, 'w', 'n', 'UTF-8');
    if fid < 0
        wb_log('error', 'FLASH', 'WRITE_FAIL', sprintf('Cannot open %s for writing', p));
        error('ESP32:FileWriteFail', 'Cannot open %s for writing.', p);
    end
    fwrite(fid, txt, 'char');
    fclose(fid);
end