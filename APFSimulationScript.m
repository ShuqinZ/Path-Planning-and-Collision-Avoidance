
F = [1, 2, 3, 4, 8, 16, 32, 64, 128, 256, 1024];
for i = 1 :  length(F)
    diaryFileName = string(F(i)) + "FLSs_APF.txt";
    diary(diaryFileName);
    tic;
    generateTestPtCld([70,70,10], 120, 120, F(i), 5);
    disp("===============================================================================================================");
    fprintf("Now rendering i = %d\n", F(i));
    renderSimulation(1, string(F(i)));
    toc;
    diary off;
end