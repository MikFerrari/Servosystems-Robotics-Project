function Q = allCombinations(q1,q2,q3)

    elements = {q1,q2,q3}; %cell array with N vectors to combine
    combinations = cell(1, numel(elements)); %set up the varargout result
    [combinations{:}] = ndgrid(elements{:});
    combinations = cellfun(@(x) x(:), combinations,'uniformoutput',false); %there may be a better way to do this
    Q = [combinations{:}]; % NumberOfCombinations by N matrix. Each row is unique.