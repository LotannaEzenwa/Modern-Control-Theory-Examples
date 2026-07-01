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

    for d = 1:numel(opts.dirs)
        thisDir = fullfile(here, opts.dirs{d});
        if ~isfolder(thisDir)
            warning('publish_all:missingDir','Skipping missing directory: %s', opts.dirs{d});
            continue
        end
        files = dir(fullfile(thisDir,'*.m'));
        for k = 1:numel(files)
            fpath = fullfile(thisDir, files(k).name);
            if is_function_file(fpath)
                continue   % skip function files -- they are utilities, not tutorials
            end
            fprintf('  publishing %s%s%s ...\n', opts.dirs{d}, filesep, files(k).name);
            try
                publish(fpath, ...
                    'format',    opts.format, ...
                    'evalCode',  logical(opts.evalCode), ...
                    'outputDir', fullfile(thisDir,'html'), ...
                    'showCode',  true, ...
                    'maxOutputLines', 30);
                close all
            catch err
                warning('publish_all:failed','  FAILED on %s: %s', files(k).name, err.message);
            end
        end
    end
    fprintf('Done. Reports are in each directory''s html/ subfolder.\n');
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
