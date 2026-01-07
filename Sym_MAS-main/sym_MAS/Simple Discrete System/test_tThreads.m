pool = parpool('Processes');

tic
for idx = 1:100
    f(idx) = parfeval(pool, @magic, 1, idx);
end

% magicResults = cell(1,100);
% for idx = 1:100
%     [completedIdx, value] = fetchNext(f);
%     magicResults{completedIdx} = value;
%     fprintf('Got the Results with Index: %d.\n', completedIdx);
% end
toc

delete(gcp('nocreate'))