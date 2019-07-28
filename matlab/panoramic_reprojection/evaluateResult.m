
for i = 1:length(Results)
    Result = Results(i);
    averagePrecision = Result.averagePrecision;
    recall = Result.recall;
    precision = Result.precision;
    classes = Result.classes;
    fov = Result.fov;
    mAP = mean(averagePrecision);
    % Plot precision/recall curve

    for j = 1:length(averagePrecision)
        rec = recall(j);
        prec = precision(j);
        figure(j)
        hold on
        plot(rec{:},prec{:})
        xlabel('Recall')
        ylabel('Precision')
        grid on
        title(sprintf('%s', string(classes(j))));
        legend(string({Results(:).fov}));
        hold off
    end
end