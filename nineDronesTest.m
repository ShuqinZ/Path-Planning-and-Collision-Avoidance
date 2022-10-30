clc; clear; close all;
util = Utility();
% startPtFile = "Point Cloud Squence/pt1379_change.ptcld";
% targetPtFiles = ["pt1547_change.ptcld", ""];

% Configurable parameters
stoptime = 1;
removeWhenCollide = true;
iterations = 1;
timeunit = 1/25;

ptClds = [];
direction = [];
distLeft = [];
waypoints = [];
collisionPt = [];
arriveNum = 0;
color = [[0,0,0];[0.7,0,0];[0,0,1];[1,0,1];[0,1,0];[0,1,1];[1,1,1];[1,1,0]];
collisions = [];
collisionsAgain = [];
collisionAgainDrones = [];
displayCell = [];
illuminationCell = [];
sizeOfIllumCell = 5;
dispCellSize = 0.1;
speedLimit = 0.6;
repulsiveRange = 3.5;
nearByIlluminationCell = floor(repulsiveRange/(dispCellSize * sizeOfIllumCell)) + 0.5;

totalSteps = 0;
% initialPts = util.loadPtCld(startPtFile);
% 
% for i = 1:size(targetPtFiles)
%     targets(i) = util.loadPtCld("Point Cloud Squence/" + targetPtFiles(i));
% end

% 3x3 matrix on the ground
% initialPts = [[0,0,0];[0,5,0];[0,10,0];[5,0,0];[5,5,0];[5,10,0];[10,0,0];[10,5,0];[10,10,0]];
% 
% % lifting up to the 3x3 matrix in the air
% ptClds(:,:,1) = [[0,0,25];[0,5,25];[0,10,25];[5,0,25];[5,5,25];[5,10,25];[10,0,25];[10,5,25];[10,10,25]]; 
% 
% % a line
% ptClds(:,:,2) = [[25,25,25];[27.5,25,25];[30,25,25];[32.5,25,25];[35,25,25];[37.5,25,25];[40,25,25];[42.5,25,25];[45,25,25]];
% 
% % a cube with one FLS sticking out
% ptClds(:,:,3) = [[45,15,15];[25,35,15];[45,35,15];[25,15,35];[25,35,35];[45,15,35];[45,35,35];[35,25,45];[25,15,15]];
% 
% % circle
% ptClds(:,:,4) = [[25,40,25];[34.65,36.5,25];[39.75,27.6,25];[38,17.5,25];[30.15,10.9,25];[19.85,10.9,25];[12,17.5,25];[10.25,27.6,25];[15.35,36.5,25]];

initialPts = readmatrix("Point Cloud Squence/90DronesGround.csv");
ptClds(:,:,1) = readmatrix("Point Cloud Squence/90DronesRise.csv");
ptClds(:,:,2) = readmatrix("Point Cloud Squence/butterfly.csv");
ptClds(:,:,3) = readmatrix("Point Cloud Squence/cat.csv");
ptClds(:,:,4) = readmatrix("Point Cloud Squence/teapot.csv");

ptCldNums = size(ptClds,3);


for i = 1:length(initialPts)
    drones(i) = Drone(i,initialPts(i,:),[0,0,0],[0,0,0],[0,0,0]);
    waypointsPerStep = [drones(i).position, drones(i).velocity, drones(i).acceleration,ptClds(i,:,1)];
end


dronesNum = length(drones);


% Start Simulation
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
            ptCld = ptClds(:,:,j);
        end
        

        step = 0;
        replanStep = 0;
        arriveNum = 0;
%         nearByDrones = [];
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
            disp(step);
            for i = 1:length(drones)
                accTime = 0;
                
                %   if the drone has arrived, skip
                if drones(i).arrived
                    continue
                end
                
%                 %   if the drone has already been removed, ignore it
%                 if drones(i).removed
%                     continue
%                 end

                % if the drone is already arrived, or just arrived, skip
%                 if all(abs(drones(i).position - drones(i).target)<=[0.002,0.002,0.002])
                if all(abs(drones(i).position - drones(i).target)<=[0.002,0.002,0.002])
                    drones(i).arrived = true;
                    arriveNum = arriveNum + 1;
                    drones(i).velocity = [0,0,0];
                    drones(i).position = drones(i).target;
                    fprintf("Drone %d arrived, %d has arrived and %d left\n", i, arriveNum, dronesNum-arriveNum);
                    %disp(step); 
                    continue
                end
                

                % Check if the drone needs to speed up or slow down
                v = norm(drones(i).velocity);
                tToSlow = v /(speedLimit * drones(i).accMax);
                distToSlow = 0.5 * v * tToSlow;
                

                %   If the distance left is no bigger than the minimum
                %   slowing down distance
                if distLeft(i) > distToSlow && ((distLeft(i) - (v * timeunit + 0.5 * speedLimit * drones(i).accMax * timeunit^2)) < (distToSlow + (timeunit * v + 0.5 * (speedLimit * drones(i).accMax)^2 * timeunit^2)))
  
                    %accTime = (sqrt(4* v^2 + 4 * drones(i).accMax * (distLeft(i) - distToSlow)) - 2 * v)/ (2 * drones(i).accMax);
                    accValue = (-(v*timeunit/(speedLimit * drones(i).accMax) + 0.5*timeunit^2)+sqrt((v*timeunit/(speedLimit * drones(i).accMax) + 0.5*timeunit^2)^2 + 2*(timeunit^2/(speedLimit * drones(i).accMax))*(distLeft(i) - v*timeunit - (v^2)/(2* speedLimit * drones(i).accMax))))...
                        /(timeunit^2/(speedLimit * drones(i).accMax));
                    %maxSpeed = v +  accTime * drones(i).accMax;
                    %maxt = maxSpeed/(drones(i).accMax);

                    %newV = drones(i).velocity + drones(i).accMax * accTime * direction(i,:) - drones(i).accMax * (timeunit-accTime) * direction(i,:);
                    %positionMoved = drones(i).velocity * accTime + 0.5 * drones(i).accMax * accTime^2 * direction(i,:) + newV * (timeunit-accTime) ...
                     %   + 0.5 * drones(i).accMax * (timeunit-accTime)^2 * direction(i,:);

                    newV = drones(i).velocity + accValue * timeunit * direction(i,:);
                    positionMoved = drones(i).velocity * timeunit + 0.5 * accValue * timeunit^2 * direction(i,:);

                    newdistToSlow = (0.5 * (v+accValue*timeunit)^2) /(speedLimit * drones(i).accMax);
                    left = distLeft(i) - norm(positionMoved);%(0.5 *accValue*timeunit^2 + v * timeunit);
                    
                elseif distLeft(i) <= distToSlow
                    accValue = min(v/timeunit, speedLimit * drones(i).accMax);
                    newV = drones(i).velocity - accValue * timeunit * direction(i,:);
                    positionMoved = 0.5 * (drones(i).velocity + newV) * timeunit;

                    newdistToSlow = (0.5 * (v-accValue*timeunit)^2) /(speedLimit * drones(i).accMax);

                elseif  v == speedLimit * drones(i).vMax
                    accValue = 0;
                    newV = drones(i).velocity;
                    positionMoved = newV * timeunit;

                    newdistToSlow = distToSlow;
                else
                    %accTime = min((drones(i).vMax - v)/drones(i).accMax, timeunit); 
                    accValue = min((speedLimit * drones(i).vMax - v)/timeunit, speedLimit * drones(i).accMax);
                    newV = drones(i).velocity + accValue * timeunit * direction(i,:);
                    positionMoved = 0.5 * accValue * direction(i,:) * timeunit^2 + drones(i).velocity * timeunit;

                    newdistToSlow = (0.5 * (v+accValue*timeunit)^2) /(speedLimit * drones(i).accMax);
                end

                distMoved = norm(positionMoved);
                distLeft(i) = distLeft(i) - distMoved;

                drones(i).acceleration = accValue * direction(i,:);

                drones(i).position = drones(i).position + positionMoved;
                drones(i).velocity = newV;
                drones(i).distTraveled = drones(i).distTraveled + distMoved;
                

                %fprintf("Moving %.4f at step %d with speed %.2f\n", distMoved, step, norm(newV));
                waypointsPerStep(i,:) = [drones(i).position, drones(i).velocity, drones(i).acceleration,drones(i).target];

%                 v = norm(drones(i).velocity);
%                 tToSlow = v /(drones(i).accMax);
%                 distToSlow = 0.5 * v * tToSlow;
%                 fprintf("the %d th drone is at (%f,%f,%f), with the speed %f, dist left %f, dist to slow %f\n", i, drones(i).position, norm(drones(i).velocity), distLeft(i), newdistToSlow);
                
%                 plot3(waypointsPerStep(:,1), waypointsPerStep(:,2), waypointsPerStep(:,3),'.','MarkerSize',3,'Color', color(mod(k,4) + 4,:));
%                 hold on;
                %fprintf("the %d th drone is at (%f,%f,%f), with the speed %f, dist left %f, dist to slow %f\n", i, drones(i).position, norm(drones(i).velocity), distLeft(i), newdistToSlow);
%                 figure(1);
%                 plot3(waypointsPerStep(:,1), waypointsPerStep(:,2), waypointsPerStep(:,3),'.','MarkerSize',3,'Color', color(5,:));
%                 hold on;
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
                        figure(1);
                        plot3(drones(i).position(1),drones(i).position(2),drones(i).position(3),'r.','MarkerSize',30);
                        fprintf("Drone %d and %d collided at [%.2f,%.2f,%.2f], step = %d\n",i,m,drones(i).position,step);
                        %dronesNum = dronesNum - 1;
                        %pause(2);
                        potentialCollide = true;
                        colDronesPerTime = [colDronesPerTime, drones(m)];
                        %nearByDrones = [nearByDrones, drones(m)];
                        

                        %find drones in neighbor illumination cells, 
                        % needs a better solution by always choosing the 
                        % drones around the current drone
%                         illumCellDIn = [];
%                         illumCellDIn(1) = floor((drones(i).position(1)/dispCellSize)/sizeOfIllumCell); 
%                         illumCellDIn(2) = floor((drones(i).position(2)/dispCellSize)/sizeOfIllumCell); 
%                         illumCellDIn(3) = floor((drones(i).position(3)/dispCellSize)/sizeOfIllumCell); 
% 
%                         for y = 1:length(drones)
%                             if y == m || y == i
%                                 continue
%                             end
%                             isNeighbor = true;
%                             for z = 1:3
%                                 isNeighbor = all(isNeighbor) && all(drones(y).position(z) > dispCellSize * (illumCellDIn(z) - sizeOfIllumCell/2));
%                                 isNeighbor = all(isNeighbor) && all(drones(y).position(z) < dispCellSize * (illumCellDIn(z) - sizeOfIllumCell/2));
%                             end
%                             if isNeighbor
%                                 nearByDrones = [nearByDrones, drones(y)];
%                             end
%                         end

                    end
                end

                if collisionDNum > 0
                    collisionDNum = collisionDNum + 1;
                    %colDronesPerTime = [colDronesPerTime, drones(i)];
%                     nearByDrones = [nearByDrones, drones(i)];
                    drones(i).removed = true;
                    %dronesNum = dronesNum -1;
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
        if ~potentialCollide
            figure(1);
            plot3(waypoints(:,1), waypoints(:,2), waypoints(:,3),'.','MarkerSize',3,'Color', color(mod(k,4) + 4,:));
            hold on;
        end
        originWaypoints = waypoints;

        % When a collision happens, we record the collision co-ordinate, the ID of
        % collision drons. Re-plan the path for the colliding drones, starting
        % form the last point cloud formation 
        while potentialCollide
            disp("Start Re-Plan");
            pause(0.5);

            replanStep = 0;
            %   save the originWaypoints in case we need to re-replan the path
            waypoints = originWaypoints;
            potentialCollide = false;

            arrivedDrones = [];
            collisionAgainDrones = [];
        
            checkPosition = [];
            for x = 1:length(colDronesPerTime)
                colDronesPerTime(x).position = waypoints(colDronesPerTime(x).ID,1:3);
                colDronesPerTime(x).arrived = false;
                colDronesPerTime(x).removed = false;
                colDronesPerTime(x).distTraveled = 0;
                colDronesPerTime(x).velocity = [0,0,0];

                arrivedDrones(x) = 0;
            end
            
            apf = APF();
            while ~all(arrivedDrones) == 1
                replanStep = replanStep + 1;
                disp(replanStep);
                if replanStep <= step
                    waypointsPerStep = waypoints(end-(step - replanStep + 1)*dronesNum + 1:end - (step - replanStep) * dronesNum,:);
                else
                    waypointsPerStep = waypoints(end-dronesNum + 1:end, :);
                end

                % needs to update the replanning drone's info based on the
                % origin path
                for i = 1:length(colDronesPerTime)
                    waypointsPerStep(colDronesPerTime(i).ID,:) = [colDronesPerTime(i).position, colDronesPerTime(i).velocity, colDronesPerTime(i).acceleration,colDronesPerTime(i).target];
                end


                for i = 1:length(colDronesPerTime)

                    checkPosition = [];
                    if colDronesPerTime(i).arrived
                        continue;
                    end
                    
%                     %   add all the replanning drone's position and all the
%                     %   near by drones to checkPosition to replan the path
%                     %   base on their position, but does not work all the
%                     %   time for near by drones are selected by the near by
%                     %   drones of the collision point. Should be better to
%                     %   check drones around.
%                     for x = 1:i-1
%                         checkPosition = [checkPosition;colDronesPerTime(x).position];
%                     end
% 
%                     for x = 1:length(nearByDrones)
%                         laststep = min(replanStep, step);
%                         checkPosition = [checkPosition;waypoints(nearByDrones(x).ID + dronesNum * (laststep - 1),1:3)];
%                     end

                    illumCellDIn = [];
                    illumCellDIn(1) = floor((colDronesPerTime(i).position(1)/dispCellSize)/sizeOfIllumCell); 
                    illumCellDIn(2) = floor((colDronesPerTime(i).position(2)/dispCellSize)/sizeOfIllumCell); 
                    illumCellDIn(3) = floor((colDronesPerTime(i).position(3)/dispCellSize)/sizeOfIllumCell); 

                    for y = 1:dronesNum
                        if y == colDronesPerTime(i).ID
                            continue
                        end
                        
                        isNeighbor = true;

                        for z = 1:3
                            isNeighbor = all(isNeighbor) && all(waypointsPerStep(y,z) > dispCellSize * (illumCellDIn(z)*sizeOfIllumCell - sizeOfIllumCell * nearByIlluminationCell));
                            isNeighbor = all(isNeighbor) && all(waypointsPerStep(y,z) < dispCellSize * (illumCellDIn(z)*sizeOfIllumCell + sizeOfIllumCell * nearByIlluminationCell));
                        end
                        if isNeighbor
                            if norm(waypointsPerStep(y,1:3)-waypointsPerStep(y,10:12)) <= 0.01
                                checkPosition = [checkPosition; [waypointsPerStep(y,1:3), y, 1]];
                            else
                                checkPosition = [checkPosition; [waypointsPerStep(y,1:3), y, 0]];
                            end
                        end
                    end

                    
                    [colDronesPerTime(i),targetExchange] = apf.getNextStep(colDronesPerTime(i),checkPosition);

                    if targetExchange && colDronesPerTime(i).targetExchangeCounter >= 50

                        fprintf("Triggered Target Exchange for drone %d [%.1f,%.1f,%.1f] and %d [%.1f,%.1f,%.1f], ",...
                            colDronesPerTime(i).ID, colDronesPerTime(i).target, checkPosition(targetExchange,4), waypointsPerStep(checkPosition(targetExchange,4),10:12));
                        if j >= 4
                            pause(0.1);
                        end
                        tmp = waypointsPerStep(checkPosition(targetExchange,4),10:12);
                        newDroneTarget = colDronesPerTime(i).target;
                        colDronesPerTime(i).target = tmp;

                        colDronesPerTime(i).targetExchangeCounter = 0;

                        addOrNot = 0;
                        for colDrone = 1:length(colDronesPerTime)
                            if colDronesPerTime(colDrone).ID == targetExchange
                                addOrNot = colDrone;
                                break;
                            end
                        end
                        if addOrNot == 0
                            newDrone = Drone(checkPosition(targetExchange,4), checkPosition(targetExchange,1:3),newDroneTarget, [0,0,0], [0,0,0]);
                            newDrone.startPt = newDrone.position;
                            colDronesPerTime = [colDronesPerTime, newDrone];
                            arrivedDrones = [arrivedDrones, 0];
                        else
                            colDronesPerTime(colDrone).arrived = 0;
                            arrivedDrones(colDrone) = 0;
                            colDronesPerTime(colDrone).target = checkPosition(targetExchange,10:12);
                        end
                        %   mark that we needs to re-replan the path 
%                         potentialCollide = true;
                    end

                    % re-write waypoints
                    waypointsPerStep(colDronesPerTime(i).ID,:) = [colDronesPerTime(i).position, colDronesPerTime(i).velocity, colDronesPerTime(i).acceleration, colDronesPerTime(i).target];

                              
                    if norm(colDronesPerTime(i).position(:)-colDronesPerTime(i).target(:)) <= 0.01
                        colDronesPerTime(i).arrived = true;
                        arrivedDrones(i) = 1;
                        fprintf("Origin Step %d, re-plan step %d, drone %d has arrived by replan, moving %.2f while origin %.2f\n", step, replanStep, colDronesPerTime(i).ID, colDronesPerTime(i).distTraveled, norm(colDronesPerTime(i).target-colDronesPerTime(i).startPt));
                        %dronesNum = dronesNum + 1;
                    end
                    %disp(steps);
                end
                % to those collide again, maintain there last position
                % before colliding
                for x = 1:length(collisionAgainDrones)
                    collisionAgainDrones(x).position = waypoints(x + dronesNum * (replanStep - 2),1:3);
                    waypointsPerStep(collisionAgainDrones(x).ID,:) = waypoints(x + dronesNum * (replanStep - 2),:);
                end
                
                waypoints(1 + dronesNum * (replanStep - 1):dronesNum * replanStep,:) = waypointsPerStep;

                 % collision detection
                for i = 1:length(colDronesPerTime)
                    colAgainDNum = 0;
                    collisionAgainDrones = [];
    
                    if colDronesPerTime(i).removed
                        continue
                    end
    
                    for m = 1:dronesNum
                        if norm(colDronesPerTime(i).position - waypoints(drones(m).ID + dronesNum * (replanStep - 1),1:3))<= 0.1 && colDronesPerTime(i).ID ~= m 
                            colAgainDNum = colAgainDNum + 1;
                            drones(m).removed = true;
                            figure(1);
                            plot3(colDronesPerTime(i).position(1),colDronesPerTime(i).position(2),colDronesPerTime(i).position(3),'r.','MarkerSize',30);
                            hold on;
                            fprintf("Drone %d and %d collided again at [%f,%f,%f] with distance %f \n", ...
                                colDronesPerTime(i).ID, m, waypoints(drones(m).ID + dronesNum * (replanStep - 1),1:3),norm(colDronesPerTime(i).position - waypoints(drones(m).ID + dronesNum * (replanStep - 1),1:3)))
                            pause(1);

                            addOrNot = true;
                            for colDrone = 1:length(colDronesPerTime)
                                if colDronesPerTime(colDrone).ID == m
                                    addOrNot = false;
                                    break;
                                end
                            end
                            if addOrNot
                                colDronesPerTime = [colDronesPerTime, drones(m)];
                                arrivedDrones = [arrivedDrones, 0];
                            else
                                colDronesPerTime(colDrone).arrived = 0;
                                arrivedDrones(colDrone) = 0;
                            end

                            needsReplan = true;

                            if removeWhenCollide
                               collisionAgainDrones = [collisionAgainDrones, drones(m)];
                            end

                            potentialCollide = true;
                        end
                    end
    
                    if colAgainDNum > 0
                        colAgainDNum = colAgainDNum + 1;
                        colDronesPerTime(i).removed = true;
                        %dronesNum = dronesNum -1;
                        collisionsAgain = [collisionsAgain,colAgainDNum];
                        arrivedDrones(i) = 1;
                    end
                end
                figure(1);
                plot3(waypointsPerStep(:,1), waypointsPerStep(:,2), waypointsPerStep(:,3),'.','MarkerSize',3,'Color', [0 0 1]);
                hold on;

                if potentialCollide
                    break
                end
    
            end


        end

        %   mark the target point
        figure(1);
        plot3(waypointsPerStep(:,1), waypointsPerStep(:,2), waypointsPerStep(:,3),'.','MarkerSize',20,'Color', color(j,:));
        hold on;

        figure;
        plot3(waypointsPerStep(:,1), waypointsPerStep(:,2), waypointsPerStep(:,3),'.','MarkerSize',20,'Color', color(j,:));
        hold on;

        for i = 1:length(colDronesPerTime)
            drones(colDronesPerTime(i).ID) = colDronesPerTime(i);
        end

        for i = 1: (stoptime/0.04)
            step = step + 1;
            waypoints = [waypoints; waypointsPerStep];
        end
        
        %disp(waypoints);
        util.saveCSV(waypoints, './pathMatrix.csv');
        totalSteps = totalSteps + size(waypoints,1) / dronesNum;

        fprintf('current point cloud takes %d steps, total %d steps\n', size(waypoints,1) / dronesNum, totalSteps);
        pause(0.01);
    end

end
disp(totalSteps);
% figure;
% h = histogram(collisions, length(dronesNum));
% xlabel('FLSs involved in the Collision','FontSize',16);
% ylabel('Collision Times','FontSize',16);
% 
% figure;
% h = histogram(collisionsAgain, length(dronesNum));
% xlabel('FLSs involved in the Collision','FontSize',16);
% ylabel('Collision Again Times','FontSize',16);


