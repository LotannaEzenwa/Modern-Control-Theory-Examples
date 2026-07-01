function publish_all(varargin)
%PUBLISH_ALL  Render every tutorial script in this repo to a report.
%   PUBLISH_ALL() runs MATLAB's built-in PUBLISH on each tutorial .m file
%   in the topic directories, evaluating the code and capturing the
%   figures, and writes one PDF report per script into a pdf/ subfolder
%   of that script's directory (the output subfolder is named after the
%   format, so HTML output would go into html/).
%
%   PUBLISH_ALL('format','html') publishes to HTML instead of PDF.
%   PUBLISH_ALL('dirs',{'Root-Locus'}) restricts to the named directories.
%   PUBLISH_ALL('evalCode',false) typesets the tutorials WITHOUT running
%       the code (fast, no figures) -- handy for a quick formatting check.
%
%   PDF quality: by default (format 'pdf') PUBLISH_ALL renders each tutorial
%   to LaTeX and compiles it with pdflatex, so equations are TRUE vector math
%   -- sharp at any zoom -- instead of the blurry bitmaps MATLAB's built-in
%   PDF export produces. This requires pdflatex on the system PATH (TeX Live,
%   MiKTeX, or MacTeX). If pdflatex is not found, PUBLISH_ALL falls back to
%   MATLAB's built-in bitmap PDF and says so. Pass 'vector',false to force the
%   built-in bitmap PDF, or 'format','html' for HTML.
%
%   Examples:
%     publish_all                       % all tutorials -> vector PDF
%     publish_all('dirs',{'Digital-Control'})
%     publish_all('vector',false)       % MATLAB's built-in bitmap PDF
%     publish_all('format','html','evalCode',true)
%
%   Helper/function files and the homework folders are skipped; only the
%   self-contained instructional scripts are published.
%
%   See also PUBLISH.

    p = inputParser;
    p.addParameter('format','pdf',@ischar);
    p.addParameter('evalCode',true,@(x)islogical(x)||isnumeric(x));
    p.addParameter('dirs',default_dirs(),@iscell);
    p.addParameter('vector',true,@(x)islogical(x)||isnumeric(x));
    p.parse(varargin{:});
    opts = p.Results;

    here = fileparts(mfilename('fullpath'));
    fprintf('Publishing tutorials (%s, evalCode=%d)\n', opts.format, opts.evalCode);

    % Make every published figure a crisp, readable dark-on-white plot,
    % regardless of the user's MATLAB desktop theme. Two coordinated pieces:
    %
    %   (1) Force a LIGHT graphics theme for the run (R2025a+). Without this,
    %       a user in MATLAB's dark theme gets black-background figures in the
    %       report. force_light_theme() sets it via the settings API and is
    %       guarded, so older releases just skip it and fall back on (2).
    %
    %   (2) Pin the graphics-root text/axis/grid colors dark and leave
    %       InvertHardcopy at its default 'on' (which whitens the figure
    %       background on publish). This keeps published LaTeX equations and
    %       figure labels black even if a startup.m changed the defaults, and
    %       guarantees a white -- never black -- background even when the theme
    %       cannot be set. A faint gray plot area and a soft-but-visible grid
    %       give the figures contrast against the white page.
    %
    % NOTE: we deliberately do NOT set InvertHardcopy 'off' or a gray figure
    % background -- under a dark desktop theme that captures the dark UI and
    % produces black figures.
    themeCleanup = force_light_theme(); %#ok<NASGU>  % restored on function exit

    rootProps = { ...
        'defaultTextColor',            [0 0 0]; ...
        'defaultAxesColor',            [0.965 0.965 0.98]; ... % faint gray plot area for contrast
        'defaultAxesXColor',           [0.15 0.15 0.15]; ...
        'defaultAxesYColor',           [0.15 0.15 0.15]; ...
        'defaultAxesZColor',           [0.15 0.15 0.15]; ...
        'defaultAxesGridColor',        [0.15 0.15 0.15]; ...
        'defaultAxesGridAlpha',        0.3; ...          % soft, always-visible grid
        'defaultAxesLineWidth',        0.75};            % a slightly heavier axis box
    oldRootVals = cellfun(@(p) get(groot,p), rootProps(:,1), 'UniformOutput', false);
    set(groot, rootProps(:,1)', rootProps(:,2)');
    restoreRoot = onCleanup(@() set(groot, rootProps(:,1)', oldRootVals')); %#ok<NASGU>

    % Decide whether to produce vector PDFs via pdflatex. Only relevant for
    % PDF output; needs pdflatex on PATH. If requested but unavailable, warn
    % once and fall back to MATLAB's built-in (bitmap) PDF export.
    useLatex = logical(opts.vector) && strcmpi(opts.format,'pdf');
    if useLatex && ~has_pdflatex()
        warning('publish_all:noPdflatex', ...
            ['pdflatex not found on PATH -- falling back to MATLAB''s bitmap ' ...
             'PDF export (equations will be lower quality). Install TeX Live / ' ...
             'MiKTeX / MacTeX, or pass ''vector'',false to silence this.']);
        useLatex = false;
    end
    if useLatex
        fprintf('Using pdflatex for vector-quality equations.\n');
    end

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
            if useLatex
                publish_pdf_via_latex(jobs(j).file, fullfile(thisDir,'pdf'), opts.evalCode);
            else
                publish(jobs(j).file, ...
                    'format',    opts.format, ...
                    'evalCode',  logical(opts.evalCode), ...
                    'outputDir', fullfile(thisDir,opts.format), ...
                    'showCode',  true, ...
                    'maxOutputLines', 30);
            end
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
    fprintf('Done (%d/%d published). Reports are in each directory''s %s/ subfolder.\n', ...
        total - numel(failures), total, opts.format);
end

% ------------------------------------------------------------------------
function c = force_light_theme()
%FORCE_LIGHT_THEME  Force a light figure graphics theme for this session.
%   Returns an onCleanup that restores the previous setting. On releases
%   without themed figures (pre-R2025a) or a different settings path, it
%   silently does nothing and the caller relies on InvertHardcopy='on'.
    c = [];
    try
        s  = settings;
        gt = s.matlab.appearance.figure.GraphicsTheme;
        gt.TemporaryValue = 'light';
        c = onCleanup(@() clearTemporaryValue(gt));
    catch
        % Older release / different settings tree -- rely on InvertHardcopy='on'.
    end
end

% ------------------------------------------------------------------------
function tf = has_pdflatex()
%HAS_PDFLATEX  True if a pdflatex executable is callable on the PATH.
    [status,~] = system('pdflatex --version');
    tf = (status == 0);
end

% ------------------------------------------------------------------------
function publish_pdf_via_latex(mfile, outDir, evalCode)
%PUBLISH_PDF_VIA_LATEX  Publish MFILE to a vector-quality PDF in OUTDIR.
%   Publishes to LaTeX (so equations stay as real, vector LaTeX rather than
%   bitmaps), then compiles the .tex with pdflatex and cleans up the
%   intermediate files, leaving just <name>.pdf. Errors if pdflatex fails.
    texPath = publish(mfile, ...
        'format',        'latex', ...
        'imageFormat',   'png', ...     % figures raster; equations stay vector
        'evalCode',      logical(evalCode), ...
        'outputDir',     outDir, ...
        'showCode',      true, ...
        'maxOutputLines', 30);
    close all

    patch_latex_report(texPath);        % wider margins, smaller code font, bigger figures

    [texDir, texName] = fileparts(texPath);
    prevDir = pwd;
    cd(texDir);                          % compile from the folder holding the images
    restore = onCleanup(@() cd(prevDir)); %#ok<NASGU>

    % If a previous PDF is still open in a viewer it is locked, and pdflatex
    % cannot overwrite it -- detect that up front and give a clear message
    % rather than pdflatex's cryptic "I can't write on file" emergency stop.
    pdfFile = fullfile(texDir, [texName '.pdf']);
    if isfile(pdfFile)
        try
            delete(pdfFile);
        catch
            error(['%s.pdf is open/locked in another program (e.g. a PDF ' ...
                   'viewer) -- close it and re-run.'], texName);
        end
    end

    % Read stdin from the null device so pdflatex can never hang waiting for
    % input, and capture BOTH passes so its verbose log does not flood the
    % console (we keep the first pass's text only to report on failure).
    nul = '/dev/null'; if ispc, nul = 'NUL'; end
    cmd = sprintf('pdflatex -interaction=nonstopmode -halt-on-error "%s.tex" < %s', texName, nul);
    [~, log1] = system(cmd);
    [~, ~]    = system(cmd);   % second pass resolves the table of contents

    if ~isfile(pdfFile)
        error('pdflatex did not produce %s.pdf. Tail of log:\n%s', texName, log_tail(log1));
    end

    % Tidy up: remove aux files, the intermediate .tex, and the figure PNGs
    % (now embedded in the PDF). Leave only the finished PDF.
    exts = {'.aux','.log','.out','.toc','.tex'};
    for e = 1:numel(exts)
        delete_quiet(fullfile(texDir, [texName exts{e}]));
    end
    figs = dir(fullfile(texDir, [texName '_*.png']));
    for k = 1:numel(figs)
        delete_quiet(fullfile(texDir, figs(k).name));
    end
end

% ------------------------------------------------------------------------
function patch_latex_report(texPath)
%PATCH_LATEX_REPORT  Tune the MATLAB-generated .tex for a nicer PDF.
%   MATLAB's LaTeX template uses default article margins, 10pt verbatim code,
%   and hardcodes 4in-wide figures -- so long code lines overflow the right
%   margin ("Overfull \hbox") and the plots look small. Widen the margins,
%   shrink the code font (which removes the overfulls), and enlarge every
%   figure. Uses only base LaTeX + geometry, so it needs no extra packages.
    txt = fileread(texPath);

    inject = strjoin({ ...
        '\usepackage[letterpaper,margin=0.75in]{geometry}', ...
        '\makeatletter', ...
        '\g@addto@macro\@verbatim{\footnotesize}', ... % smaller code font -> no overfull
        '\makeatother'}, newline);
    txt = strrep(txt, '\begin{document}', [inject newline '\begin{document}']);

    % Enlarge every included figure to nearly the (now wider) text width.
    txt = regexprep(txt, '(\\includegraphics)\s*(\[[^\]]*\])?', '$1[width=6.5in]');

    fid = fopen(texPath, 'w');
    if fid < 0, error('Could not rewrite %s', texPath); end
    closer = onCleanup(@() fclose(fid)); %#ok<NASGU>
    fwrite(fid, txt);
end

% ------------------------------------------------------------------------
function delete_quiet(f)
%DELETE_QUIET  Delete F if it exists, ignoring errors.
    if isfile(f)
        try, delete(f); catch, end
    end
end

% ------------------------------------------------------------------------
function s = log_tail(txt)
%LOG_TAIL  Last few lines of a pdflatex log, for error messages.
    lines = strsplit(txt, newline);
    n = numel(lines);
    s = strjoin(lines(max(1,n-12):n), newline);
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
