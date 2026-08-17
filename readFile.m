function data = readFile(filename)
    % Read the numerical values from the specified file
    % Input: filename - Name of the file containing the data
    % Output: data - Struct containing the parsed data

    f = fopen(filename, 'r');
    if f == -1
        error('Error: Unable to open file %s', filename);
    end

    % Initialize a structure to hold the data
    data = struct();

    % Read the file line by line
    while ~feof(f)
        line = strtrim(fgetl(f)); 
        if isempty(line) || startsWith(line, '%')
            continue;
        end
     
        parts = strsplit(line, '%'); % Split line at the '%' symbol
        if length(parts) == 2
            value = str2double(strtrim(parts{1})); % Convert the first part to a number
            label_with_unit = strtrim(parts{2});            % Get the label from the second part
            label = strtrim(strsplit(label_with_unit,'('){1}); %supress the unit
            
            if ~isempty(label) %if the label is not valid
                label = strrep(label, '+', '_plus');
                label = strrep(label, '-', '_minus');
                data.(label) = value;
            end
        end
    end
    fclose(f);
end
