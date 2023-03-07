function publishFunc( varargin )
% PUBLISHFUNC - publish the HTML documentation of a toolbox
% 
% Syntax:
%       PUBLISHFUNC
%       PUBLISHFUNC('FunctionName')
%       PUBLISHFUNC('FunctionName','evalcode',bool)
% 
% Description:
%       This function generates an HTML documentation for a single given function (If no
%	   function name is given, you will be ask to pick the m-file via UI). After
%       calling this function, the function documentation will be opened for inspection.
%
%       The PUBLISHFUNC calls the function FUNHELP2MARKUP to generate marked-up
%       m-files out of the headers of the functions in the toolbox. This
%       automatically generated m-files, together with user-defined m-files, are
%       then published as HTML files. 
%
% Input Arguments:
%       *Optional Input Arguments:*
%       -'FunctionName':    Name (as string) of function to be published (case sensitive);
%
%       *Parameter Input Arguments:*
%       -'evalcode':        Boolean option to evaluate code when publishing
%                           the documentation of the toolbox to be released.;
%                           [{false} / true]
%
% See Also: 
%       publishHelp, funHelp2MarkUp


%% Parse the inputs

minInputs = 0;
maxInputs = 3;
narginchk(minInputs,maxInputs)

p = inputParser;

defaultFileName = '';
addOptional(p,'file',defaultFileName,@ischar);

defaultEvalCode = false;
addParameter(p,'evalcode',defaultEvalCode,@islogical);

p.FunctionName = 'publishFunc';

parse(p,varargin{:});

if isempty(p.Results.file) == 1
    % select file via GUI
    cd(fileparts(fileparts(mfilename('fullpath'))));
    [FileName,PathName] = uigetfile('*.m','Select the MATLAB code file');
else
    FileName = p.Results.file;
    [~,FileName] = fileparts(FileName);        
    PathName = fileparts(which(p.Results.file)); 
end

%% setup directories

% file path
filePath = fullfile(PathName,FileName);
if ~strcmp(filePath(end-1:end) , '.m')
    filePath = [filePath,'.m'];
end

% root directory
path = fileparts(fileparts(fileparts(mfilename('fullpath'))));

% doc directory
if ~strncmpi(path,PathName,length(path))
   error('The specified function is not part of the "AROC" toolbox!'); 
end

doc_dir = fullfile(path,'documentation','generateDoc');

% source and html directory
source_dir = fullfile(path,'documentation','doc','source');
html_dir = fullfile(path,'documentation','doc','html');


%% Publishing Options
pubOpts.format = 'html';
pubOpts.maxHeight = 800;
pubOpts.maxWidth = 600;
pubOpts.evalCode = p.Results.evalcode;
pubOpts.stylesheet = fullfile(path,'documentation','generateDoc','myStyleSheet.xsl');
pubOpts.createThumbnail = false;


%% Get toolbox-specific data

year = 2018;
website = sprintf('%s',...
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


%% Footer
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
	website, ...
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


%% Convert to Markup

markupOpts.OutputDir    = fullfile(source_dir , 'functions');
markupOpts.Footer       = footer_;
markupOpts.Files = {filePath};

funRef_ = funHelp2MarkUp( markupOpts );


%% Generate the html-files from the marked-up help m-files

fprintf('\n\n\nGenerating the html-helpfile for function %s.\n\n', FileName);
fprintf('\tSource directory:\n\t\t%s\n\n', source_dir);
fprintf('\tOutput directory:\n\t\t%s\n\n', html_dir);
fprintf('Start publishing %s.\n\n\n\n', FileName);

% publishing
pubOpts.outputDir = html_dir;

srcPath = fullfile(source_dir , 'functions',funRef_.file);
pf_ = publish( srcPath , pubOpts );
		
fprintf('Published ''%s'' to:\n\t%s\n', funRef_.name,pf_);
fprintf('\n\n\nFinished publishing %s.\n\n\n\n', FileName);


%% Close all figures wenn evalcode == 1 
if p.Results.evalcode == true
    close all
end


%% Open published file
web(pf_);

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
