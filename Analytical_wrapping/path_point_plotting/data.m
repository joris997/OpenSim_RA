clear; close; clc;
A = dlmread('outputFile.txt');

numOutputs = 4;
for i = 1:size(A,1)/numOutputs
    results(i).npoints = A(1+(i-1)*numOutputs,1);
    results(i).time = A(2+(i-1)*numOutputs,1);
    results(i).interval = A(3+(i-1)*numOutputs,1);
    results(i).muscleLength = A(4+(i-1)*numOutputs,1:end-1);
    results(i).fiberLength = A(5+(i-1)*numOutputs,1:end-1);
    results(i).tendonLength = A(6+(i-1)*numOutputs,1:end-1);
end

A = dlmread('outputFileWrap.txt');
wResults(1).npoints = 0;
wResults(1).time = A(2,1);
wResults(1).interval = A(3,1);
wResults(1).muscleLength = A(4,1:end-1);
wResults(1).fiberLength = A(5,1:end-1);
wResults(1).tendonLength = A(6,1:end-1);