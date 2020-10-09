clear; close; clc;
A = dlmread('outputFile.txt');

for i = 1:size(A,1)/3
    results(i).npoints = A(1+(i-1)*3,1);
    results(i).time = A(2+(i-1)*3,1);
    results(i).points = A(3+(i-1)*3,1:end-1);
end
 
wResults.npoints = 0;
wResults.time = 5.01892;
wResults.points = [1.15133
1.19131
1.17967
1.20512
1.1841
1.2177
1.03506
0.848622
0.73034
0.675355
0.651249
0.80848
1.35097
0.843693
1.04843
1.12989
0.826803
0.717943
0.669975
0.64882
0.639151
0.811934
1.34498
0.836026
1.05354]';