tic
%define function you want to optimize
% func = load_test_function(18);
%number of variables in a the function (=number of genes)
N_var = 6;

%lists of upper and lower boundaries starting from x
lower_bounds = [1 0.01 0 500 0.01 0];
upper_bounds = [5 0.1 0 1000 0.1 0];

N_pop = 50;    %number of chromosomes
sel = 0.2;    %selection rate

M_kept = 2*ceil(sel*N_pop/2);               %number of chromosomes surviving natural selection
mut_rate = 0.8;                             %mutation rate
n_muts = ceil(mut_rate*N_var*(N_pop-1));    %number of mutations
best_costs = [];
best_chroms = [];
tolerance_interval = 10e-3;                 %when cost enters this interval, the GA is stopped
min_gens = 500;                              %--> can't stop the GA before min_gens generations has passed

%rng(3);


r = Q3_interp';
t = (1:length(Q3_interp))*0.001;

r = r(1:10:end);
t = t(1:10:end);

Pparams = load('system_params');
Pparams = Pparams.system_params;


%creates N_pop random chromosomes with N_var genes each
pop = rand(N_pop,N_var);    %initial population

%un-normalization of the values in pop matrix
    for j = 1:N_var
        pop(:,j) = (upper_bounds(j)-lower_bounds(j))*pop(:,j) + lower_bounds(j);
    end

%creates .txt file in order to save data about the progression of the algorithm
% currentFolder = pwd;
% txtFiles = dir(sprintf('%s\\Data\\*.txt',currentFolder)); 
% numTxtFiles = length(txtFiles);
% log_file = fopen(sprintf('%s\\Data\\ProgressionLog%d.txt',currentFolder,numTxtFiles),'w');
% fprintf(log_file, 'Best cost and coordinates of the minimum for each generation\n');

i = 1;
go_on = true;
%iterations through generations
while go_on       %termination condition
    
    %cost function evaluation
    cost = Inf(size(pop,1),1);
    for n = 1:size(pop,1)
        cost(n,:) = test_controller(pop(n,:),Pparams,r,t);
    end
    [costs,indexes] = sort(cost);
    disp(costs(1));
    disp(pop(indexes(1),:));
    
    %ranks results and chromosomes, gathering statistical data
    pop = pop(indexes(1:M_kept),:);     %performing natural selection in the meantime
    popToSave = pop;
    costsToSave = costs(1:M_kept);
%     s = sprintf('[iteration: %d,       cost: %f,       point:    ( %f , %f )]\n', i, costs(1), pop(1,1),pop(1,2));
%     fprintf(s);
%     fprintf(log_file,s);
    
    best_costs = [best_costs; costs(1)];
    best_chroms = [best_chroms; pop(1,:)];
          
    %pairing --> strategy: rank weighting with probabilities
    num_matings = ceil(M_kept/2);
    mothers = zeros(1,num_matings); %contains indexes of mother-chromosomes
    fathers = zeros(1,num_matings); %contains indexes of father-chromosomes
    probabilities = zeros(1,M_kept);
    cum_probabilities = zeros(1,M_kept);
    probabilities(1) = M_kept/sum(1:M_kept);
    cum_probabilities(1) = probabilities(1);
    
    for k = 2:M_kept
        probabilities(k) = (M_kept-k+1)/sum(1:M_kept);
        cum_probabilities(k) = cum_probabilities(k-1)+probabilities(k);
    end %k
    
    cursor1 = cum_probabilities(1) + rand(1,num_matings)*(1-cum_probabilities(1));
    cursor2 = cum_probabilities(1) + rand(1,num_matings)*(1-cum_probabilities(1));
    
    for k = 1:num_matings
        for l = 1:(M_kept-1)
            if cursor1(k) <= cum_probabilities(l+1) && cursor1(k) > cum_probabilities(l)
                mothers(k) = l;
            end
            if cursor2(k) <= cum_probabilities(l+1) && cursor2(k) > cum_probabilities(l)
                fathers(k) = l;
            end
        end %l
    end %k
    %end pairing
    
    
    %mating --> strategy: single-point crossover with basic blending method (p.59)
    %(does not allow generations of variables outside the bounds
    %already set by the parents)
    
    crossings = ceil(rand(1,num_matings)*N_var);
    random_multiplier = rand(1, num_matings);
    
    for k = 1:num_matings
        mo = pop(mothers(k),:);
        fa = pop(fathers(k),:);
        
        new_var_1 = mo(crossings(k)) - random_multiplier(k)*(mo(crossings(k))-fa(crossings(k)));
        new_var_2 = fa(crossings(k)) + random_multiplier(k)*(mo(crossings(k))-fa(crossings(k)));
        
        if crossings(k) ~= 1 && crossings(k) ~= N_var
            off1 = [mo(1:crossings(k)-1) new_var_1 fa(crossings(k)+1:end)];
            off2 = [fa(1:crossings(k)-1) new_var_2 mo(crossings(k)+1:end)];
        end
        
        if crossings(k) == 1
            off1 = [new_var_1 fa(crossings(k)+1:end)];
            off2 = [new_var_2 mo(crossings(k)+1:end)];
        end
        
        if crossings(k) == N_var && N_var ~= 1
            off1 = [fa(1:crossings(k)-1) new_var_1];
            off2 = [mo(1:crossings(k)-1) new_var_2];
        end
        
        pop = [pop; off1; off2];
        
    end %k
    %end mating
    
    
    %mutations
    for k = 1:n_muts
        i_row = ceil(1 + rand*(M_kept-1)); %the best chromosome isn't mutated due to elitism
        i_col = ceil(N_var*rand);
        pop(i_row,i_col) = lower_bounds(i_col) + rand*(upper_bounds(i_col)-lower_bounds(i_col));
    end %k
    %end mutations
    
    %check termination conditions
    if i >= min_gens && (abs(best_costs(i) - best_costs(i-1))) <= tolerance_interval...
                     && (abs(best_costs(i) - best_costs(i-2))) <= tolerance_interval...
                     && (abs(best_costs(i) - best_costs(i-3))) <= tolerance_interval...
                     && (abs(best_costs(i) - best_costs(i-4))) <= tolerance_interval...
                     && (abs(best_costs(i) - best_costs(i-5))) <= tolerance_interval...
        go_on = false;
    end
    
    i = i + 1;
    
end %while

%Visualization of the results and best_cost trend
% fprintf('The algorithms has converged after %d generations\n', i-1);
figure('Name','ConvergencePlot','NumberTitle','off');
tiledlayout(2,1);
nexttile;
title('Best cost trend over generations');
plot(1:i-1, best_costs);
nexttile;
title('Best chromosome over generations');
hold on
plot(best_chroms(:,1), best_chroms(:,2), '-'); %N_var = 2
plot(best_chroms(end,1), best_chroms(end,2), 'ro');
%plot(best_chroms(:,1), '-o'); %N_var = 1
% save_figure(currentFolder,'Data','ConvergencePlot'); %saves ConvergencePlot into .fig file
hold off

%saves WorkspaceVariables and PolupationResults into .mat and .xls files
% xlsFiles = dir(sprintf('%s\\Data\\*.xls',currentFolder)); 
% numXlsFiles = length(xlsFiles);
% dataToBeSaved = table(popToSave,costsToSave);
% writetable(dataToBeSaved, sprintf('%s\\Data\\PopulationResults%d.xls',currentFolder,numXlsFiles));
% matFiles = dir(sprintf('%s\\Data\\*.mat',currentFolder)); 
% numMatFiles = length(matFiles);
% save(sprintf('%s\\Data\\PopulationResults%d.mat',currentFolder,numMatFiles));
% 
% %Ends time count and stores elapsed time into variable t
% t = toc;
% fprintf("CPU time %f\n", t);
% fprintf(log_file,"\nCPU time %f\n", t);
% 
% %closes fprintf stream to .txt file
% fclose(log_file);


