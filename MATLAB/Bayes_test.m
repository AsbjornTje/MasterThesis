clear all;
clc; 
close all;

%Generate Data

rng default

%first to base points for both classes

grnpop = mvnrnd([1,0],eye(2),10);
redpop = mvnrnd([0,1],eye(2),10);

plot(grnpop(:,1),grnpop(:,2),'go')
hold on
plot(redpop(:,1),redpop(:,2),'ro')
hold off

%generate 100 datapoints for each class

redpts = zeros(100,2);grnpts = redpts;
for i = 1:100
    grnpts(i,:) = mvnrnd(grnpop(randi(10),:),eye(2)*0.02);
    redpts(i,:) = mvnrnd(redpop(randi(10),:),eye(2)*0.02);
end

figure
plot(grnpts(:,1),grnpts(:,2),'go')
hold on
plot(redpts(:,1),redpts(:,2),'ro')
hold off

%Prepare Data for Classification
%Put the data into one matrix, and make a vector grp that labels the class of each point. 1 indicates the green class, and -1 indicates the red class.

cdata = [grnpts;redpts];
grp = ones(200,1);
grp(101:200) = -1;

%Prepare Cross-Validation
%Set up a partition for cross-validation. This step fixes the train and test sets that the optimization uses at each step.

c = cvpartition(200,'KFold',10);

%Prepare Variables for Bayesian Optimization
%Set up a function that takes an input z = [rbf_sigma,boxconstraint] and returns the cross-validation loss value of z. Take the components of z as positive, log-transformed variables between 1e-5 and 1e5. Choose a wide range, because you don't know which values are likely to be good.

sigma = optimizableVariable('sigma',[1e-5,1e5],'Transform','log');
box = optimizableVariable('box',[1e-5,1e5],'Transform','log');

%Objective Function
%This function handle computes the cross-validation loss at parameters [sigma,box]. For details, see kfoldLoss.
%bayesopt passes the variable z to the objective function as a one-row table.

minfn = @(z)kfoldLoss(fitcsvm(cdata,grp,'CVPartition',c,...
    'KernelFunction','rbf','BoxConstraint',z.box,...
    'KernelScale',z.sigma));

%Optimize Classifier
%Search for the best parameters [sigma,box] using bayesopt. For reproducibility, choose the 'expected-improvement-plus' acquisition function. 
% The default acquisition function depends on run time, and so can give varying results.

results = bayesopt(minfn,[sigma,box],'IsObjectiveDeterministic',true,...
    'AcquisitionFunctionName','expected-improvement-plus')


%Obtain the best estimated feasible point from the XAtMinEstimatedObjective property or by using the bestPoint function. 
% By default, the bestPoint function uses the 'min-visited-upper-confidence-interval' criterion. 
% For details, see the Criterion name-value argument of bestPoint.

results.XAtMinEstimatedObjective
z = bestPoint(results)

%Use the best point to train a new, optimized SVM classifier.

SVMModel = fitcsvm(cdata,grp,'KernelFunction','rbf', ...
    'KernelScale',z.sigma,'BoxConstraint',z.box);

%To visualize the support vector classifier, predict scores over a grid.

d = 0.02;
[x1Grid,x2Grid] = meshgrid(min(cdata(:,1)):d:max(cdata(:,1)), ...
    min(cdata(:,2)):d:max(cdata(:,2)));
xGrid = [x1Grid(:),x2Grid(:)];
[~,scores] = predict(SVMModel,xGrid);

%Plot the classification boundaries.

figure
h(1:2) = gscatter(cdata(:,1),cdata(:,2),grp,'rg','+*');
hold on
h(3) = plot(cdata(SVMModel.IsSupportVector,1) ,...
    cdata(SVMModel.IsSupportVector,2),'ko');
contour(x1Grid,x2Grid,reshape(scores(:,2),size(x1Grid)),[0 0],'k');
legend(h,{'-1','+1','Support Vectors'},'Location','Southeast');

%Evaluate Accuracy on New Data
%Generate and classify new test data points.

grnobj = gmdistribution(grnpop,.2*eye(2));
redobj = gmdistribution(redpop,.2*eye(2));

newData = random(grnobj,10);
newData = [newData;random(redobj,10)];
grpData = ones(20,1);  % green = 1
grpData(11:20) = -1; % red = -1

v = predict(SVMModel,newData);

%Compute the misclassification rates on the test data set.

L = loss(SVMModel,newData,grpData)

%See which new data points are correctly classified. Circle the correctly classified points in red, and the incorrectly classified points in black.

h(4:5) = gscatter(newData(:,1),newData(:,2),v,'mc','**');

mydiff = (v == grpData); % Classified correctly

for ii = mydiff % Plot red squares around correct pts
    h(6) = plot(newData(ii,1),newData(ii,2),'rs','MarkerSize',12);
end

for ii = not(mydiff) % Plot black squares around incorrect pts
    h(7) = plot(newData(ii,1),newData(ii,2),'ks','MarkerSize',12);
end
legend(h,{'-1 (training)','+1 (training)','Support Vectors', ...
    '-1 (classified)','+1 (classified)', ...
    'Correctly Classified','Misclassified'}, ...
    'Location','Southeast');
hold off