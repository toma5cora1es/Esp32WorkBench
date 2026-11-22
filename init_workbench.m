function paths = init_workbench()
%INIT_WORKBENCH Register core/instruments directories and initialize workbench state.

    rootDir = fileparts(mfilename('fullpath'));
    setenv('ESP32_WB_ROOT', rootDir);

    dirs = {'core', 'instruments', 'experiments'};
    for k = 1:numel(dirs)
        d = fullfile(rootDir, dirs{k});
        if isfolder(d)
            addpath(genpath(d));
        end
    end

    wb_config('workbench');
    lvls = wb_log('set_level', 'all');
    paths = wb_paths();

    wb_log('normal', 'INIT', 'READY', sprintf('Root=%s | Log=%s', paths.root, upper(strjoin(lvls, ','))));
end