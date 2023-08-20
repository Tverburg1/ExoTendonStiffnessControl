function readSerialData(src, ~)

    data = readline(src);
    line = strsplit(data ,{': ', ', '},'CollapseDelimiters',true);

    src.experimentData.Data(end+1) = [line(2:2:end)];    
    src.experimentData.Count = src.experimentData.Count + 1;
end

