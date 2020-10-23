clear; close; clc;
A = dlmread('outputFile.txt');

numOutputs = 9;
for i = 1:size(A,1)/numOutputs
    results(i).time = A(1+(i-1)*numOutputs,1);
    results(i).npoints = A(2+(i-1)*numOutputs,1);
    results(i).interval = A(3+(i-1)*numOutputs,1);
    
    results(i).muscleMaxForce = A(4+(i-1)*numOutputs,1);
    results(i).optFiberLength= A(5+(i-1)*numOutputs,1);
    results(i).tendonSlackLength = A(6+(i-1)*numOutputs,1);
    
    results(i).muscleLength = A(7+(i-1)*numOutputs,1:end-1);
    results(i).fiberLength = A(8+(i-1)*numOutputs,1:end-1);
    results(i).tendonLength = A(9+(i-1)*numOutputs,1:end-1);
end