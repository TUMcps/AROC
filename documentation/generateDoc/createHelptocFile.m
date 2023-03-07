function createHelptocFile(pathHtml,files)
% Create the helptoc.xml file that describes the structure of the
% html-documentation. 

    % create top-level structure
    docNode = com.mathworks.xml.XMLUtils.createDocument('toc');
    docRootNode = docNode.getDocumentElement;
    docRootNode.setAttribute('version','2.0');
    docRootNode.setAttribute('image','$toolbox/matlab/icons/book_mat.gif');
    
    docToolboxNode = docNode.createElement('tocitem');
    docToolboxNode.setAttribute('target','product_page.html');
    docToolboxNode.appendChild(docNode.createTextNode('AROC'));
    
    docFunctionNode = docNode.createElement('tocitem');
    docFunctionNode.setAttribute('image','HelpIcon.FUNCTION');
    docFunctionNode.appendChild(docNode.createTextNode('Function Reference'));
    docFunctionNode.setAttribute('target','functions_index.html');
    
    % add the html-pages for all functions
    for i = 1:length(files)
        docFunctionNode = addFilePages(docNode,docFunctionNode,files{i});
    end
    
    % add the created nodes as child-nodes to the root elements
    docToolboxNode.appendChild(docFunctionNode);
    docRootNode.appendChild(docToolboxNode);
    
    % write xml-document to xml-file
    xmlFileName = fullfile(pathHtml,'helptoc.xml');
    xmlwrite(xmlFileName,docNode);
    type(xmlFileName);
end


% Auxiliary Functions -----------------------------------------------------

function parentNode = addFilePages(docNode,parentNode,files)

   % create child node
   [path,name,~] = fileparts(files.name);
   
   docChildNode = docNode.createElement('tocitem');
   if strcmp(name,'auxiliary') || strcmp(name,'settings')
        [~,nameDir,~] = fileparts(path);
        docChildNode.setAttribute('target',[nameDir,'_',name,'_index.html']);
   else
        docChildNode.setAttribute('target',[name,'_index.html']);
   end
   docChildNode.setAttribute('image','HelpIcon.FUNCTION');
   docChildNode.appendChild(docNode.createTextNode(convertFolderName(name)));
   
   % add nodes for all subdirectories
   for i = 1:length(files.subdirs)
       docChildNode = addFilePages(docNode,docChildNode,files.subdirs{i});
   end
   
   % add nodes for all files
   for i = 1:length(files.files)
       [path,nameTemp,~] = fileparts(files.files{i});
       [~,dirName,~] = fileparts(path);
       docTempNode = docNode.createElement('tocitem');
       if dirName(1) == '@'
            docTempNode.setAttribute('target',[dirName nameTemp,'help.html']);
       else
            docTempNode.setAttribute('target',[nameTemp,'help.html']);
       end
       docTempNode.setAttribute('image','HelpIcon.FUNCTION');
       docTempNode.appendChild(docNode.createTextNode(nameTemp));
       docChildNode.appendChild(docTempNode);
   end
   
   % add child node to parent node
   parentNode.appendChild(docChildNode);
end
