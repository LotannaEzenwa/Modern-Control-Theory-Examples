function publish_all(varargin)
%PUBLISH_ALL  Render every tutorial script in this repo to a report.
%   PUBLISH_ALL() runs MATLAB's built-in PUBLISH on each tutorial .m file
%   in the topic directories, evaluating the code and capturing the
%   figures, and writes one HTML report per script into an html/ subfolder
%   of that script's directory.
%
%   PUBLISH_ALL('format','pdf') publishes to PDF instead of HTML.
%   PUBLISH_ALL('dirs',{'Root-Locus'}) restricts to the named directories.
%   PUBLISH_ALL('evalCode',false) typesets the tutorials WITHOUT running
%       the code (fast, no figures) -- handy for a quick formatting check.
%
%   Examples:
%     publish_all                       % all tutorials -> HTML
%     publish_all('dirs',{'Digital-Control'})
%     publish_all('format','pdf','evalCode',true)
%
%   Helper/function files and the homework folders are skipped; only the
%   self-contained instructional scripts are published.
%
%   See also PUBLISH.

    p = inputParser;
    p.addParameter('format','html',@ischar);
    p.addParameter('evalCode',true,@(x)islogical(x)||isnumeric(x));
    p.addParameter('dirs',default_dirs(),@iscell);
    p.parse(varargin{:});
    opts = p.Results;

    here = fileparts(mfilename('fullpath'));
    fprintf('Publishing tutorials (%s, evalCode=%d)\n', opts.format, opts.evalCode);

    % Pin the graphics-root colors to a professional dark-on-light scheme for
    % the duration of the run, then restore the user's defaults via onCleanup.
    % Two problems this solves:
    %   (1) Published LaTeX equations and figure titles/labels are captured
    %       from offscreen text objects whose color comes from these root
    %       defaults; if a startup.m changed them they can render white and
    %       become invisible -- so we pin all text/axis colors dark.
    %   (2) On a white HTML page a pure-white figure has no visible edge, so
    %       the plots "disappear". We give the figure a soft gray frame around
    %       a white plot area (the classic report look), keep a visible grid,
    %       and -- critically -- turn OFF InvertHardcopy, because publish/print
    %       otherwise forces the captured background back to white regardless
    %       of the FigureColor we set.
    rootProps = { ...
        'defaultTextColor',            [0 0 0]; ...
        'defaultAxesColor',            [1 1 1]; ...      % white plot area: data stays legible
        'defaultAxesXColor',           [0.15 0.15 0.15]; ...
        'defaultAxesYColor',           [0.15 0.15 0.15]; ...
        'defaultAxesZColor',           [0.15 0.15 0.15]; ...
        'defaultAxesGridColor',        [0.15 0.15 0.15]; ...
        'defaultAxesGridAlpha',        0.25; ...         % soft, always-visible grid
        'defaultAxesLineWidth',        0.75; ...         % a slightly heavier axis box
        'defaultFigureColor',          [0.93 0.93 0.95]; ... % soft gray frame vs. the white page
        'defaultFigureInvertHardcopy', 'off'};           % keep that gray when published
    oldRootVals = cellfun(@(p) get(groot,p), rootProps(:,1), 'UniformOutput', false);
    set(groot, rootProps(:,1)', rootProps(:,2)');
    restoreRoot = onCleanup(@() set(groot, rootProps(:,1)', oldRootVals')); %#ok<NASGU>

    % Build the full work list up front so the progress bar knows the total.
    % (Function files and the homework folders are excluded here, not mid-loop,
    % so the count and the bar reflect only files we actually publish.)
    jobs = struct('dir',{},'file',{});
    for d = 1:numel(opts.dirs)
        thisDir = fullfile(here, opts.dirs{d});
        if ~isfolder(thisDir)
            warning('publish_all:missingDir','Skipping missing directory: %s', opts.dirs{d});
            continue
        end
        files = dir(fullfile(thisDir,'*.m'));
        for k = 1:numel(files)
            if is_function_file(fullfile(thisDir, files(k).name))
                continue   % skip function files -- they are utilities, not tutorials
            end
            jobs(end+1) = struct('dir',opts.dirs{d},'file',files(k).name); %#ok<AGROW>
        end
    end
    total = numel(jobs);
    if total == 0
        fprintf('No tutorial scripts found to publish.\n');
        return
    end

    startDir  = pwd;
    restoreCd = onCleanup(@() cd(startDir)); %#ok<NASGU>  % return home even on error
    failures  = {};
    for j = 1:total
        thisDir = fullfile(here, jobs(j).dir);
        label   = sprintf('%s%s%s', jobs(j).dir, filesep, jobs(j).file);
        print_progress(j-1, total, label);      % show the file about to be published
        cd(thisDir);                 % run from inside the folder so a local file
                                     % (e.g. Intro.m) shadows same-named files on the path
        try
            publish(jobs(j).file, ...
                'format',    opts.format, ...
                'evalCode',  logical(opts.evalCode), ...
                'outputDir', fullfile(thisDir,'html'), ...
                'showCode',  true, ...
                'maxOutputLines', 30);
            close all
        catch err
            failures{end+1} = sprintf('%s: %s', label, err.message); %#ok<AGROW>
        end
        cd(startDir);
    end
    print_progress(total, total, 'complete');
    fprintf('\n');   % close off the progress-bar line

    if ~isempty(failures)
        fprintf('%d file(s) failed to publish:\n', numel(failures));
        for i = 1:numel(failures)
            fprintf('  FAILED %s\n', failures{i});
        end
    end
    fprintf('Done (%d/%d published). Reports are in each directory''s html/ subfolder.\n', ...
        total - numel(failures), total);
end

% ------------------------------------------------------------------------
function print_progress(done, total, label)
%PRINT_PROGRESS  Overwrite-in-place command-window progress bar.
%   Uses a carriage return (no newline) so repeated calls update a single
%   line. LABEL is padded/truncated to a fixed width so a shorter filename
%   can't leave leftover characters from a longer previous one.
    barWidth = 30;
    frac  = done / total;
    nfill = round(frac * barWidth);
    bar   = [repmat('#',1,nfill), repmat('.',1,barWidth-nfill)];

    label   = char(label);
    labWidth = 42;
    if numel(label) > labWidth
        label = ['...', label(end-labWidth+4:end)];      % keep the tail (filename)
    else
        label = [label, repmat(' ',1,labWidth-numel(label))];
    end
    fprintf('\r[%s] %3.0f%% (%d/%d) %s', bar, frac*100, done, total, label);
end

% ------------------------------------------------------------------------
function d = default_dirs()
%DEFAULT_DIRS  The instructional directories, in suggested reading order.
    d = {'Intro', ...
         'Mathematical Models', ...
         'Transient and Steady-State', ...
         'Root-Locus', ...
         'Frequency-Response', ...
         'PID Controllers', ...
         'State-Space', ...
         'Servo-and-Tracking', ...
         'Digital-Control', ...
         'Nonlinear-Systems', ...
         'System-Identification', ...
         'Kalman-Filtering', ...
         'Case-Studies'};
end

% ------------------------------------------------------------------------
function tf = is_function_file(fpath)
%IS_FUNCTION_FILE  True if the first executable line of fpath is a function.
    tf = false;
    fid = fopen(fpath,'r');
    if fid<0, return; end
    cleaner = onCleanup(@() fclose(fid));
    while ~feof(fid)
        line = strtrim(fgetl(fid));
        if isempty(line) || startsWith(line,'%')
            continue   % skip blank lines and comments
        end
        tf = startsWith(line,'function');
        return
    end
end
