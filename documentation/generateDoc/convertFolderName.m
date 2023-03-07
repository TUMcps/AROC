function name = convertFolderName(name)
% Folder names are in the documentation displayed with uppercase letters
% and with spaces between the single words

    if name(1) ~= '@'
        
        name_ = upper(name(1));
        idx = isstrprop(name,'upper') ;

        for i = 2:length(name)
            if idx(i) == 1 && idx(i-1) ~= 1
               name_ = [name_,' ',name(i)];
            else
               name_ = [name_,name(i)];
            end
        end

        name = name_;
    end
end