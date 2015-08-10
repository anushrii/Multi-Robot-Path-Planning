function [boundary, blocks] = read_map_file(filename)
%% load the map and read line by line and convert to int

fid = fopen(filename);
tline = fgetl(fid);
blocks = [];
while ischar(tline)
    if (strcmp(tline,'')==0)
        words = strsplit(tline);
        indicator = words(1);
        if(strcmp(indicator(1),'#')==0)
            content = words(2:end);
            content = cellfun(@str2num, content);
            if(strcmp(indicator,'block')==1)
                % remove block text
                blocks = [blocks;content];
            elseif (strcmp(indicator,'boundary')==1)
                boundary =content;
            end
            
        end
    end
    tline = fgetl(fid);
end
% keep within boundary
for i = 1:size(blocks,1)
    for j = 1:3
        blocks(i,j) = enclose2minmax(blocks(i,j),boundary(j), boundary(j+3));
        blocks(i,j+3) = enclose2minmax(blocks(i,j+3),boundary(j), boundary(j+3));
    end
end
fclose(fid);
end

function val = enclose2minmax(val, min_val, max_val)
    val = min(max(min_val,val), max_val);
end