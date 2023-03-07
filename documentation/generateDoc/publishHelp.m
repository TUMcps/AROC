function publishHelp(varargin)
% PUBLISHHELP - publish the HTML documentation of a toolbox
% 
% Syntax:
%       PUBLISHHELP
%       PUBLISHHELP('evalcode',bool)
% 
% Description:
%       This function generates an HTML documentation for a given toolbox. After
%       calling this function, the toolbox documentation can be found under
%       "Supplemental Software" in the MATLAB documentation by typing "doc" in
%       the command window.
%
%       The PUBLISHHELP calls the function FUNHELP2MARKUP to generate marked-up
%       m-files out of the headers of the functions in the toolbox. This
%       automatically generated m-files, together with user-defined m-files, are
%       then published as HTML files. The function CREATEHELPTOCFILE is called to
%       update the "helptoc.xml" file necessary for the HTML documentation. The
%       "info.xml" file has to be generated manually.
%
%       "info.xml", "helptoc.xml" and the published HTML files form all together the HTML
%       documentation of the toolbox.
%
% Input Arguments:
%       *Parameter Input Arguments:*
%       -'evalcode':        Boolean option to evaluate code when publishing
%                           the documentation of the toolbox to be released.;
%                           [{false} / true]
%
% See Also: 
%       pubslishFunc, funHelp2MarkUp


%% Parse the inputs
minInputs = 0;
maxInputs = 2;
narginchk(minInputs,maxInputs)

p = inputParser;

defaultEvalCode = false;
addParameter(p,'evalcode',defaultEvalCode,@islogical);

p.FunctionName = 'publishHelp';

parse(p,varargin{:});


%% Setup directories

% root directory
path = fileparts(fileparts(fileparts(mfilename('fullpath'))));

% doc directory
doc_dir = strcat(path,filesep,'documentation',filesep,'doc');


%% Publishing Options
pubOpts.format          = 'html';
pubOpts.maxHeight       = 800;
pubOpts.maxWidth        = 600;
pubOpts.evalCode        = p.Results.evalcode;
pubOpts.stylesheet      = strcat(path,filesep,'documentation',filesep,'generateDoc',filesep, 'myStyleSheet.xsl');
pubOpts.createThumbnail = false;


% add doc folder to the path
addpath(doc_dir);
dirs = getSubdirectories(doc_dir);
for j = 1:length(dirs)
    addpath(dirs{j}); 
end


%% Get toolbox-specific data

folders = excludedFolders();

year = 2018;
web = sprintf('%s',...
	'%        <a href="https://tumcps.github.io/AROC/">Website</a>');

logo = sprintf('%s\n%s\n%s\n%s\n%s\n%s\n%s\n%s\n%s',...
    '%   <td style="background-color:#ffffff; border:0; width:25%; vertical-align:middle; text-align:center">',...
    '%             <img src="img/logoAroc.png" alt="logoAroc" height="40px">',...
    '%      </td>', ...
    '%   <td style="background-color:#ffffff; border:0; width:25%; vertical-align:middle; text-align:center">',...
    '%      <img src="img/logoCora.png" alt="logoCora" height="40px"></td>', ...
    '%   <td style="background-color:#ffffff; border:0; width:25%; vertical-align:middle; text-align:center">',...
    '%      <img src="img/logoChair.png" alt="logoChair" height="40px"></td>', ...
    '%   <td style="background-color:#ffffff; border:0; width:25%; vertical-align:middle; text-align:center">',...
    '%      <img src="img/logoTum.png" alt="logoTum" height="40px"></td>');

year_ = datestr(now, 'yyyy');
year = num2str(year);
if ~strcmp(year_,year)
    year_ = [year '-' year_];
end
footer_ = sprintf('%s\n', ...
	'%%', ...
	'% <html>', ...
	'%   <hr>', ...
	['%   <p class="copy">&copy; ' year_ ' I6 Technische Universit&auml;t M&uuml;nchen'], ...
	'%        <tt class="minicdot">&#149;</tt>', ...
	web, ...
	'%        <tt class="minicdot">&#149;</tt>', ...
    '%        <a href="file:txts/LICENSE.txt">License</a>', ...
    '%   </p>', ...
    '% <div>', ...
    '% <table>', ...
    '%  <tr>', ...
    logo,...
    '%  </tr>', ...
    '% </table>',...
    '% </div>', ...
	'% </html>');



%% Setup directories

% html directory, where all generated html-files are saved to
html_dir = fullfile(doc_dir , 'html');

if exist( html_dir, 'dir' ) ~= 7
	mkdir( html_dir )
else
	% delete existing html files
	delete( fullfile(html_dir , '*.html') );
	% delete existing png files
	delete( fullfile(html_dir , '*.png') )
end

% source directory, where all marked-up m-files are located
source_dir = fullfile(doc_dir , 'source');

if exist( source_dir, 'dir' ) ~= 7
	mkdir( source_dir )
	mkdir( fullfile(source_dir , 'functions') )
else
	% delete existing function m-files
	delete( fullfile(source_dir , 'functions', '*.m') );
end


%% Remove all directories that should not be included from the list

% replace all exluded folders by the full filepath
repo_dir = fileparts(fileparts(doc_dir));

for i = 1:length(folders)
   folders{i} = fullfile(repo_dir,folders{i}); 
end

% get all subdirectories of the repository
[fun_files_rec,fun_files] = getSubdirectoriesTree(repo_dir,folders);


%% Generate index files

source_dir = fullfile(doc_dir , 'source');

for i = 1:length(fun_files_rec)
    generateIndexFile(source_dir,fun_files_rec{i},footer_);
end


%% Extract help comments from function files

% options for funHelp2MarkUp function
markupOpts.OutputDir    = source_dir;
markupOpts.Footer       = footer_;

% find all files that shall be marked up
if ~isfield(markupOpts, 'Files') || ~iscell(markupOpts.Files)
	markupOpts.Files = {};
end

for iF_ = 1:length(fun_files)
	fun_ = fun_files{iF_};
	switch exist( fun_ ) %#ok
		case 2  % if fun_ is an m-file
			s_ = which( fun_ );  % 'which' displays the full path of m-file
			markupOpts.Files = [markupOpts.Files; s_];
		case 7  % if fun_ is a folder
			s_ = what( fun_ );
            % 'what' lists all Matlab relevant files in folder (not subfolders) and returns a struct
			if ~strcmp(s_.path(end),filesep) % Ensure a path divider at the end
				s_.path(end+1) = filesep;
            end
			for iS_ = 1:numel(s_.m) % s_.m is the array in the struct with m-files
				markupOpts.Files = [markupOpts.Files; [s_.path s_.m{iS_}]]; % read all m-files in folder
			end
		otherwise
			fprintf(2, 'Could not find file/directory of:\n\t''%s''\n',fun_);
			fprintf(2, 'Will be skipped!\n');
	end
end

fprintf('\n\nStart automatic help generation\n\n\n\n');

% create the marked-up files for all files in the toolbox
funHelp2MarkUp( markupOpts );


%% Generate helptoc file

createHelptocFile(html_dir,fun_files_rec);



%% Generate the html-files from the marked-up help m-files

fprintf('\n\n\nGenerating the html-helpfiles.\n\n');
fprintf('\tSource directory:\n\t\t%s\n\n', source_dir);
fprintf('\tOutput directory:\n\t\t%s\n\n', html_dir);
fprintf('Start publishing.\n\n\n\n');


% publishing
pubOpts.outputDir = html_dir;

content = dir(source_dir);

for i = 1:length(content)
    
    nameTemp = content(i).name;

    if ~content(i).isdir && ~strcmp(nameTemp,'..') && ~strcmp(nameTemp,'.')
    
        filePath = fullfile(source_dir,nameTemp);
        
        pf_ = publish( filePath , pubOpts );

        [~, filename, ext] = fileparts( pf_ );
        fprintf('Published ''%s'' to:\n\t%s\n', filePath, [filename ext]);
    end
end

fprintf('\n\n\nFinished publishing.\n\n\n\n')


%% Build the docsearch database files

% necessary to enable searching in the help browser for the created help
% files
fprintf('Building the search database ... \n')
builddocsearchdb( html_dir );
fprintf('\n\n\n\n\t----> Finished publishing!\n\n\n\n')

% add documentation folder to the MATLAB path
addpath(genpath(doc_dir));


%% Close all figures wenn evalcode == 1 
if p.Results.evalcode == true
    close all
end

end



% Auxiliary Functions -----------------------------------------------------

function [dirs,dirsFlat] = getSubdirectoriesTree(path,folders)
% create a cell array with all subdirectories of the root directory
% specified in path

    % get list of all items in the directory
    content = dir(path);
    dirs = {};
    dirsFlat = {};
    
    % loop over all elements
    for i = 1:length(content)
        
       nameTemp = content(i).name;
        
       % check if element is a directory
       if content(i).isdir && ~strcmp(nameTemp,'..') && ~strcmp(nameTemp,'.')  && ~strcmp(nameTemp,'.git') 
          
          % check if the folder was exluded by the user
          pathNew = fullfile(path,nameTemp);          
          addFolder = 1;
          
          for j = 1:length(folders)
            len = length(folders{j});
            if strncmpi(pathNew,folders{j},len) 
                addFolder = 0;
                break;
            end
          end

          if addFolder
              % store the directory name
              temp.name = pathNew;
              
              % find all matlab files in the folder
              allFiles = what(pathNew);
              mFiles = allFiles.m;
              if ~isempty(mFiles)
                 files = cell(length(mFiles),1);
                 for j = 1:length(mFiles)
                     files{j} = fullfile(pathNew,mFiles{j});
                 end
                 temp.files = files;
              else
                 temp.files = {};   
              end

              % call to recursive function
              [subdirs,subdirsFlat] = getSubdirectoriesTree(pathNew,folders);
              temp.subdirs = subdirs;
              dirsFlat = [dirsFlat, pathNew, subdirsFlat];

              % update list
              dirs{1,end+1} = temp;
          end
       end
    end
end

function dirs = getSubdirectories(path)
% create a cell array with all subdirectories of the root directory
% specified in path

    % get list of all items in the directory
    content = dir(path);
    dirs = {};
    
    % loop over all elements
    for i = 1:length(content)
        
       nameTemp = content(i).name;
        
       % check if element is a directory
       if content(i).isdir && ~strcmp(nameTemp,'..') && ~strcmp(nameTemp,'.')  && ~strcmp(nameTemp,'.git') 
          
          % store the directory name
          pathNew = fullfile(path,nameTemp);
          dirs{1,end+1} = pathNew;
           
          % call to recursive function
          dirs = [dirs, getSubdirectories(pathNew)];
       end
    end
end

function generateIndexFile(path_source,files,footer)
% create a recurive cell array storing all subdirectories and files of the
% directory structures

    % open the index file
    [path,name,~] = fileparts(files.name);
    if strcmp(name,'auxiliary') || strcmp(name,'settings')
        [~,nameDir,~] = fileparts(path);
        fileName = fullfile(path_source,[nameDir,'_',name,'_index.m']);   
    else
        fileName = fullfile(path_source,[name,'_index.m']);   
    end
    fid = fopen(fileName,'w');

    % write header to the file
    fprintf(fid,'%%%% %s\n\n', convertFolderName(name));
    
    % write list of subfolders to the file
    if ~isempty(files.subdirs)
        fprintf(fid,'%%%% %s\n%%\n', 'Directories');
        for i = 1:length(files.subdirs)
           [~,dirName,~] = fileparts(files.subdirs{i}.name);
           fprintf(fid,'%% * <.\\%s_index.html |%s|>\n',dirName,convertFolderName(dirName));
        end
        fprintf(fid,'\n');
    end

    % write list of files to the file
    if ~isempty(files.files)
        fprintf(fid,'%%%% %s\n%%\n', 'Files');
        for i = 1:length(files.files)
           [~,fileName,~] = fileparts(files.files{i});
           if name(1) == '@'
               fprintf(fid,'%% * <.\\%shelp.html |%s|>\n',[name,fileName],fileName);
           else
               fprintf(fid,'%% * <.\\%shelp.html |%s|>\n',fileName,fileName);
           end
        end 
        fprintf(fid,'\n');
    end
    
    % write footer to the file
    lines = splitlines(footer);
    for i = 1:length(lines)
       if ~isempty(lines{i})
           fprintf(fid,'%s\n',lines{i});
       end
    end
    
    % close file
    fclose(fid);
    
    % generate index files for all subdirectories
    for i = 1:length(files.subdirs)
        generateIndexFile(path_source,files.subdirs{i},footer);
    end
end

