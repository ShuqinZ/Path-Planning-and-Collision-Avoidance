clc; clear; close all;
util = Utility();
% startPtFile = "Point Cloud Squence/pt1379_change.ptcld";
% targetPtFiles = ["pt1547_change.ptcld", ""];

% Configurable parameters
stoptime = 1;
removeWhenCollide = true;
iterations = 2;
ptCldNums = 4;
timeunit = 1/25;
ptCldSeq = [1,2,3,4];

ptClds = [];
direction = [];
distLeft = [];
waypoints = [];
collisionPt = [];
arriveNum = 0;
color = [[0,0,0];[0.7,0,0];[0,0,1];[1,0,1];[0,1,0];[0,1,1];[1,1,1];[1,1,0]];
collisions = [];

displayCell = [];
illuminationCell = [];
sizeOfIllumCell = 5;
dispCellSize = 0.1;

% initialPts = util.loadPtCld(startPtFile);
% 
% for i = 1:size(targetPtFiles)
%     targets(i) = util.loadPtCld("Point Cloud Squence/" + targetPtFiles(i));
% end

% 3x3 matrix on the ground
initialPts = [[0,0,0];[0,5,0];[0,10,0];[5,0,0];[5,5,0];[5,10,0];[10,0,0];[10,5,0];[10,10,0]];

% lifting up to the 3x3 matrix in the air
ptClds(:,:,1) = [[0,0,25];[0,5,25];[0,10,25];[5,0,25];[5,5,25];[5,10,25];[10,0,25];[10,5,25];[10,10,25]]; 

% a line
ptClds(:,:,2) = [[25,25,25];[27.5,25,25];[30,25,25];[32.5,25,25];[35,25,25];[37.5,25,25];[40,25,25];[42.5,25,25];[45,25,25]];

% a cube with one FLS sticking out
ptClds(:,:,3) = [[45,15,15];[25,35,15];[45,35,15];[25,15,35];[25,35,35];[45,15,35];[45,35,35];[35,25,45];[25,15,15]];

% circle
ptClds(:,:,4) = [[25,40,25];[34.65,36.5,25];[39.75,27.6,25];[38,17.5,25];[30.15,10.9,25];[19.85,10.9,25];[12,17.5,25];[10.25,27.6,25];[15.35,36.5,25]];

for i = 1:length(initialPts)
    drones(i) = Drone(i,initialPts(i,:),[0,0,0],[0,0,0],[0,0,0]);
end

waypointsPerStep = initialPts;

dronesNum = length(drones);

for k = 1:iterations


    startPtCld = 2;
    endPtCld = ptCldNums;
    % the first iteration needs to have lifting process
    if k == 1
        startPtCld = 1;
    end

    % the last iteration needs to have landing process
    if k == iterations
        endPtCld = ptCldNums + 2;
    end


    for j = startPtCld:endPtCld

        waypoints = [];

        if j == ptCldNums + 1
            ptCld = ptClds(:,:,1);
        elseif j == ptCldNums + 2
            ptCld = initialPts;
        else
            ptCld = ptClds(:,:,ptCldSeq(j));
        end
        

        step = 0;
        arriveNum = 0;
        nearByDrones = [];
        colDrones = [];
        colDronesPerTime = [];
        potentialCollide = false;

        for i = 1:length(drones)
            drones(i).startPt = drones(i).position;
            drones(i).target = ptCld(i,:);
            drones(i).arrived = false;
            drones(i).velocity = [0,0,0];
            drones(i).distTraveled = 0;
        end

        % set the moving direction of each drones
        for i = 1:size(ptCld)
            direction(i,:) = util.differential(drones(i).position, ptCld(i,:));
            distLeft(i) = norm(drones(i).position - ptCld(i,:));
        end
        
        while arriveNum ~= dronesNum
            step = step + 1;
            
            for i = 1:length(drones)
                accTime = 0;
                
                %   if the drone has arrived, skip
                if drones(i).arrived
                    continue
                end
                
                %   if the drone has already been removed, ignore it
                if drones(i).removed
                    continue
                end

                % if the drone is already arrived, or just arrived, skip
                if all(abs(drones(i).position - drones(i).target)<=[0.0001,0.0001,0.0001])
                    drones(i).arrived = true;
                    arriveNum = arriveNum + 1;
                    drones(i).velocity = [0,0,0];
                    fprintf("Drone %d arrived, at speed (%f,%f,%f)\n", i, drones(i).velocity)
                    %disp(step); 
                    continue
                end
                

                % Check if the drone needs to speed up or slow down
                v = norm(drones(i).velocity);
                tToSlow = v /(drones(i).accMax);
                distToSlow = 0.5 * v * tToSlow;
                

                %   If the distance left is no bigger than the minimum
                %   slowing down distance
                if distLeft(i) > distToSlow && ((distLeft(i) - (v * timeunit + 0.5 * drones(i).accMax * timeunit^2)) < (distToSlow + (timeunit * v + 0.5 * drones(i).accMax^2 * timeunit^2)))
  
                    accTime = (sqrt(4* v^2 + 4 * drones(i).accMax * (distLeft(i) - distToSlow)) - 2 * v)/ (2 * drones(i).accMax);

                    maxSpeed = v +  accTime * drones(i).accMax;
                    maxt = maxSpeed/(drones(i).accMax);
                    newdistToSlow = 0.5 * maxSpeed* maxt;
                    left = distLeft(i) - 0.5 * (v+maxSpeed) * accTime;

                    newV = drones(i).velocity + drones(i).accMax * accTime * direction(i,:) - drones(i).accMax * (timeunit-accTime) * direction(i,:);
                    positionMoved = drones(i).velocity * accTime + 0.5 * drones(i).accMax * accTime^2 * direction(i,:) + newV * (timeunit-accTime) ...
                        + 0.5 * drones(i).accMax * (timeunit-accTime)^2 * direction(i,:);
                    
                elseif distLeft(i) <= distToSlow
                    accTime = min(v/drones(i).accMax,timeunit);
                    newV = drones(i).velocity - drones(i).accMax * accTime * direction(i,:);
                    positionMoved = 0.5 * (drones(i).velocity + newV) * accTime;

                elseif  v == drones(i).vMax
                    accTime = 0;
                    newV = drones(i).velocity;
                    positionMoved = newV * timeunit;
                else
                    accTime = min((drones(i).vMax - v)/drones(i).accMax, timeunit); 
                    newV = drones(i).velocity + drones(i).accMax * accTime * direction(i,:);
                    positionMoved = 0.5 * (drones(i).velocity + newV) * accTime + newV * (timeunit-accTime);
                end

                distMoved = norm(positionMoved);
                distLeft(i) = distLeft(i) - distMoved;
                
                drones(i).position = drones(i).position + positionMoved;
                drones(i).velocity = newV;
                drones(i).distTraveled = drones(i).distTraveled + distMoved;
                

                %fprintf("Moving %.4f at step %d with speed %.2f\n", distMoved, step, norm(newV));
                waypointsPerStep(i,:) = drones(i).position;

%                 v = norm(drones(i).velocity);
%                 tToSlow = v /(drones(i).accMax);
%                 distToSlow = 0.5 * v * tToSlow;

                %fprintf("the %d th drone is at (%f,%f,%f), with the speed %f, dist left %f, dist to slow %f\n", i, drones(i).position, norm(drones(i).velocity), distLeft(i), distToSlow);
                
                plot3(waypointsPerStep(:,1), waypointsPerStep(:,2), waypointsPerStep(:,3),'.','MarkerSize',10,'Color', color(mod(k,4) + 4,:));
                hold on;
            end
            
            % collision detection
            for i = 1:length(drones)
                collisionDNum = 0;
                if drones(i).removed
                    continue
                end

                for m = (i + 1):length(drones)
                    if norm(drones(i).position - drones(m).position)<= 0.1 && ~drones(m).removed
                        collisionDNum = collisionDNum + 1;

                        %   marke all colliding drones
                        drones(m).removed = true;
                        plot3(drones(i).position(1),drones(i).position(2),drones(i).position(3),'r.','MarkerSize',30);
                        disp("Collided!")
                        dronesNum = dronesNum - 1;
                        %pause(2);
                        potentialCollide = true;
                        colDronesPerTime = [colDronesPerTime, drones(m)];
                        %nearByDrones = [nearByDrones, drones(m)];
                        

                        %find drones in neighbor illumination cells
                        illumCellDIn = [];
                        illumCellDIn(1) = floor((drones(i).position(1)/dispCellSize)/sizeOfIllumCell); 
                        illumCellDIn(2) = floor((drones(i).position(2)/dispCellSize)/sizeOfIllumCell); 
                        illumCellDIn(3) = floor((drones(i).position(3)/dispCellSize)/sizeOfIllumCell); 

                        for y = 1:length(drones)
                            if y == m || y == i
                                continue
                            end
                            isNeighbor = true;
                            for z = 1:3
                                isNeighbor = all(isNeighbor) && all(drones(y).position(z) > dispCellSize * (illumCellDIn(z) - sizeOfIllumCell/2));
                                isNeighbor = all(isNeighbor) && all(drones(y).position(z) < dispCellSize * (illumCellDIn(z) - sizeOfIllumCell/2));
                            end
                            if isNeighbor
                                nearByDrones = [nearByDrones, drones(y)];
                            end
                        end

                    end
                end

                if collisionDNum > 0
                    collisionDNum = collisionDNum + 1;
                    %colDronesPerTime = [colDronesPerTime, drones(i)];
                    nearByDrones = [nearByDrones, drones(i)];
                    drones(i).removed = true;
                    dronesNum = dronesNum -1;
                    collisions = [collisions,collisionDNum];
                end
            end

            if ~removeWhenCollide
                for i = 1:length(drones)
                    drones(i).removed = false;
                end
                potentialCollide = false;
            end

            waypoints = [waypoints; waypointsPerStep];
        end

        % When a collision happens, we record the collision co-ordinate, the ID of
        % collision drons. Re-plan the path for the colliding drones, starting
        % form the last point cloud formation 
        if potentialCollide

            replanStep = 0;
            arrivedDrones = 0;
            collisionAgainDrones = [];
        
            checkPosition = [];
            for x = 1:length(colDronesPerTime)
                colDronesPerTime(x).position = colDronesPerTime(x).startPt;
                colDronesPerTime(x).removed = false;
                colDronesPerTime(x).distTraveled = 0;
                colDronesPerTime(x).velocity = 0;
            end
            
            apf = APF();
            while length(colDronesPerTime) ~= arrivedDrones
                replanStep = replanStep + 1;
                if replanStep == 1
                    pause(0.1);
                end
                disp(replanStep);
                if replanStep <= step
                    waypointsPerStep = waypoints(end-(step - replanStep + 1)*9 + 1:end - (step - replanStep) * 9,:);
                end

                for i = 1:length(colDronesPerTime)

                    checkPosition = [];
                    if colDronesPerTime(i).arrived
                        continue;
                    end
                    
                    for x = 1:i-1
                        checkPosition = [checkPosition;colDronesPerTime(x).position];
                    end

                    for x = 1:length(nearByDrones)
                        laststep = min(replanStep, step);
                        checkPosition = [checkPosition;waypoints(nearByDrones(x).ID + 9 * (laststep - 1),:)];
                    end

                    colDronesPerTime(i) = apf.getNextStep(colDronesPerTime(i),checkPosition);

                    % re-write waypoints
                    waypointsPerStep(colDronesPerTime(i).ID,:) = colDronesPerTime(i).position;
                              
                    if norm(colDronesPerTime(i).position(:)-colDronesPerTime(i).target(:)) <= 0.01
                        colDronesPerTime(i).arrived = true;
                        arrivedDrones = arrivedDrones + 1;
                        disp("Re-planned and arrived!")
                        fprintf("Drone %d has arrived by replan, moving %.2f while origin %.2f\n", colDronesPerTime(i).ID, colDronesPerTime(i).distTraveled, norm(colDronesPerTime(i).target-colDronesPerTime(i).startPt));
                        dronesNum = dronesNum + 1;
                    end
                    %disp(steps);
                end
                % to those collide again, maintain there last position
                % before colliding
                for x = 1:length(collisionAgainDrones)
                    collisionAgainDrones(x).position = waypoints(x + 9 * (replanStep - 2),:);
                    waypointsPerStep(collisionAgainDrones(x),:) = waypoints(x + 9 * (replanStep - 2),:);
                end
                
                waypoints(1 + 9 * (replanStep - 1):9 * replanStep,:) = waypointsPerStep;

                 % collision detection
                for i = 1:length(colDronesPerTime)
                    colAgainDNum = 0;
                    collisionAgainDrones = [];
    
                    if colDronesPerTime(i).removed
                        continue
                    end
    
                    for m = 1:length(drones)
                        if norm(colDronesPerTime(i).position - waypoints(drones(m).ID + 9 * (replanStep - 1),:))<= 0.1 && ~drones(m).removed
                            colAgainDNum = colAgainDNum + 1;
                            drones(m).removed = true;
                            plot3(colDronesPerTime(i).position(1),colDronesPerTime(i).position(2),colDronesPerTime(i).position(3),'r.','MarkerSize',30);
                            disp("Collided Again!")
                            if m <= length(colDronesPerTime)
                                arrivedDrones = arrivedDrones + 1;
                            end
                            pause(2);

                            if removeWhenCollide
                               collisionAgainDrones = [collisionAgainDrones, nearByDrones(m)];
                            end

                            potentialCollide = true;
                        end
                    end
    
                    if colAgainDNum > 0
                        colAgainDNum = colAgainDNum + 1;
                        colDronesPerTime(i).removed = true;
                        dronesNum = dronesNum -1;
                        collisionsAgain = [collisionsAgain,colAgainDNum];
                        collisionAgainDrones = [collisionAgainDrones, colDronesPerTime(i)];
                        arrivedDrones = arrivedDrones + 1;
                    end
                end
                plot3(waypointsPerStep(:,1), waypointsPerStep(:,2), waypointsPerStep(:,3),'.','MarkerSize',10,'Color', [0 0 1]);
                hold on;
    
                if ~removeWhenCollide
                    for i = 1:length(nearByDrones)
                        nearByDrones(i).removed = false;
                    end
                    
                    for i = 1:length(colDronesPerTime)
                        colDronesPerTime(i).removed = false;
                    end
                    potentialCollide = false;
                end
                
            end


        end

        potentialCollide = false;

        %   mark the target point
        plot3(waypointsPerStep(:,1), waypointsPerStep(:,2), waypointsPerStep(:,3),'.','MarkerSize',20,'Color', color(j,:));
        hold on;

        for i = 1:length(colDronesPerTime)
            drones(colDronesPerTime(i).ID) = colDronesPerTime(i);
        end

        for i = 1: (stoptime/0.04)
            step = step + 1;
            waypoints = [waypoints; waypointsPerStep];
        end

        %util.saveCSV(waypoints);

        pause(0.1)
    end
end

figure(2);
h = histogram(collisions, length(initialPts));
xlabel('FLSs involved in the Collision','FontSize',16);
ylabel('Collision Times','FontSize',16);



