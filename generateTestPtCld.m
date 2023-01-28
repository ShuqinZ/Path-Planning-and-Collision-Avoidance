function dispatcherPos = generateTestPtCld(center, diameter1, diameter2, numberOfPts, distance)
    clappingAngle = 2 * pi / numberOfPts;
    ptCld1 = zeros(numberOfPts,3);
    ptCld2 = zeros(numberOfPts,3);
    for i = 1 : numberOfPts
        point1 = center + [diameter1/2 * sin((i-1) * clappingAngle), diameter1/2 * cos((i-1) * clappingAngle), 0];
        ptCld1(i,:) = point1;

        point2 = center + [diameter2/2 * sin((i-1) * clappingAngle), diameter2/2 * cos((i-1) * clappingAngle), distance];
        if mod(numberOfPts,2) == 0
            index = mod(numberOfPts/2+i,numberOfPts);
        elseif mod(numberOfPts,2) == 1
            index = mod(i + 1,numberOfPts);
        end
        if ~index
            index = numberOfPts;
        end

        ptCld2(index,:) = point2;
    end

    folder = "./Point Cloud Squence/";
    surfix = ".ptcld";
    ptCldFile1 = folder + "Test1Ptcld" + surfix;

    ptCldFile2 = folder + "Test2Ptcld" + surfix;

    fID1 = fopen(ptCldFile1,'w');
    fID2 = fopen(ptCldFile2,'w');
    fprintf(fID1,"OFF\n%d 0 0\n",numberOfPts);
    fprintf(fID2,"OFF\n%d 0 0\n",numberOfPts);

    for j = 1 : numberOfPts

        fprintf(fID1,"%.2f %.2f %.2f\n",ptCld1(j,:));

        fprintf(fID2,"%.2f %.2f %.2f\n",ptCld2(j,:));
    end
    
    fclose(fID1);
    fclose(fID2);
    
    dispatcherPos = zeros(numberOfPts, 3);
    dispatcherPos(:,1:2) = ptCld1(:,1:2);
    for i = 1 : numberOfPts
        fprintf("[%.2f, %.2f, %.2f]\n,", ptCld1(i,1:2),0);
        
    end
end

