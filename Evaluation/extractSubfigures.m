 hAx = findobj('type', 'axes');
 for iAx = 1:length(hAx)
    fNew = figure;
    hNew = copyobj(hAx(iAx), fNew);
    % Change the axes position so it fills whole figure
    set(hNew, 'pos', [0.1 0.1 0.8 0.8])
 end