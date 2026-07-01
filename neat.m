function neat(varargin)
%NEAT  Gather the published tutorial PDFs into one top-level folder.
%   PUBLISH_ALL writes one PDF report per tutorial into a pdf/ subfolder of
%   each topic directory. NEAT collects all of those scattered PDFs into a
%   single top-level folder so they can be browsed (or zipped/shared) in one
%   place, WITHOUT having to dig through every topic directory.
%
%   NEAT() moves every <topic>/pdf/*.pdf into a top-level 'pdf/' folder.
%   NEAT('dest','Published') uses a different top-level folder name.
%   NEAT('copy',true) copies instead of moving (leaves the originals).
%   NEAT('dirs',{'Root-Locus'}) restricts to the named topic directories.
%
%   To avoid clobbering when two directories publish a same-named file, the
%   moved PDF is prefixed with its topic directory, e.g. Root-Locus/pdf/
%   LeadCompensation.pdf becomes pdf/Root-Locus_LeadCompensation.pdf.
%
%   Examples:
%     publish_all           % create the per-directory PDFs
%     neat                  % collect them into ./pdf
%     neat('copy',true)     % same, but keep the per-directory copies
%
%   See also PUBLISH_ALL.

    p = inputParser;
    p.addParameter('dest','pdf',@ischar);
    p.addParameter('copy',false,@(x)islogical(x)||isnumeric(x));
    p.addParameter('dirs',default_dirs(),@iscell);
    p.parse(varargin{:});
    opts = p.Results;

    here    = fileparts(mfilename('fullpath'));
    destDir = fullfile(here, opts.dest);
    if ~isfolder(destDir)
        mkdir(destDir);
    end

    action = 'Moving';
    if opts.copy, action = 'Copying'; end
    fprintf('%s published PDFs into %s%s%s\n', action, opts.dest, filesep, '');

    moved = 0;
    for d = 1:numel(opts.dirs)
        pdfDir = fullfile(here, opts.dirs{d}, 'pdf');
        if ~isfolder(pdfDir)
            continue   % nothing published for this topic yet
        end
        files = dir(fullfile(pdfDir, '*.pdf'));
        for k = 1:numel(files)
            src = fullfile(pdfDir, files(k).name);
            % Prefix with the topic dir (spaces -> underscores) so same-named
            % tutorials from different folders don't overwrite each other.
            prefix = strrep(opts.dirs{d}, ' ', '_');
            dst = fullfile(destDir, sprintf('%s_%s', prefix, files(k).name));
            if opts.copy
                [ok,msg] = copyfile(src, dst);
            else
                [ok,msg] = movefile(src, dst);
            end
            if ok
                moved = moved + 1;
            else
                warning('neat:failed', 'Could not place %s: %s', files(k).name, msg);
            end
        end
    end

    fprintf('Done. %d PDF(s) now in %s%s\n', moved, opts.dest, filesep);
end

% ------------------------------------------------------------------------
function d = default_dirs()
%DEFAULT_DIRS  The instructional directories PUBLISH_ALL renders into.
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
