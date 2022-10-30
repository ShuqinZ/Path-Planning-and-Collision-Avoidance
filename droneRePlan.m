clc; clear; close all;
util = Utility();
waypoints = readmatrix('./pathMatrix.csv');
steps = 0;

% % 3x3 matrix on the ground
% ptClds(:,:,1) = [[0,0,0];[0,5,0];[0,10,0];[5,0,0];[5,5,0];[5,10,0];[10,0,0];[10,5,0];[10,10,0]];
% 
% ptClds(:,:,2) = [[0,0,25];[0,5,25];[0,10,25];[5,0,25];[5,5,25];[5,10,25];[10,0,25];[10,5,25];[10,10,25]]; 
% 
% % a line
% ptClds(:,:,3) = [[25,25,25];[27.5,25,25];[30,25,25];[32.5,25,25];[35,25,25];[37.5,25,25];[40,25,25];[42.5,25,25];[45,25,25]];
% 
% % a cube with one FLS sticking out
% ptClds(:,:,4) = [[45,15,15];[25,35,15];[45,35,15];[25,15,35];[25,35,35];[45,15,35];[45,35,35];[35,25,45];[25,15,15]];
% 
% % circle
% ptClds(:,:,5) = [[25,40,25];[34.65,36.5,25];[39.75,27.6,25];[38,17.5,25];[30.15,10.9,25];[19.85,10.9,25];[12,17.5,25];[10.25,27.6,25];[15.35,36.5,25]];
% 
% % 3x3 matrix on the ground
% ptClds(:,:,6) = [[0,0,0];[0,5,0];[0,10,0];[5,0,0];[5,5,0];[5,10,0];[10,0,0];[10,5,0];[10,10,0]];

timeUnit = 1/25;

startStep = 246603;
droneID = 1;
dronesNum = 90;
% change its position to 20 steps before's
%waypoints(droneID + floor(startStep/9)*9,1:3) = [34.31783678,20.34115797,20.34108161];

% waypoints(droneID + floor(startStep/dronesNum)*dronesNum,1:3) = [30,22,22];


apf = APF();


potentialWayPts = [];

replan = 0;
stepToTarget = [];

replanDrones = [];

replanSteps = [0];
arrivedDrones = [0];

for j = 2:size(waypoints,1)
    if replan
        break;
    end
    lastWaypointsPerStep = waypoints((j-1)*dronesNum - dronesNum + 1: (j-1)*dronesNum,:);
    waypointsPerStep = waypoints(j*dronesNum - dronesNum + 1: j*dronesNum,:);

    if norm(lastWaypointsPerStep(dronesNum,10:12) - waypointsPerStep(dronesNum,10:12)) ~= 0
        figure(1);
        plot3(waypointsPerStep(:,1), waypointsPerStep(:,2), waypointsPerStep(:,3),'.','MarkerSize',20,'Color', [0 0 0]);
        hold on;

        figure;
        plot3(waypointsPerStep(:,1), waypointsPerStep(:,2), waypointsPerStep(:,3),'.','MarkerSize',20,'Color', [0 0 0]);
        hold on;
    end
    

    for i = 1:dronesNum
        v = lastWaypointsPerStep(i,4:6);
        acc = lastWaypointsPerStep(i,7:9);
        position = lastWaypointsPerStep(i,1:3);
        if steps == startStep
            acc = acc/2;
        end

        if acc == [0,0,0]
            position = position + (norm(v)*timeUnit + 0.5 * norm(acc) * timeUnit^2) * v / norm(v);
        else
            position = position + (norm(v)*timeUnit + 0.5 * norm(acc) * timeUnit^2) * acc / norm(acc);
        end

        steps = steps + 1;
        disp(steps);
%         fprintf('%d step %d, drone %d at [%.4f, %.4f, %.4f], supposed to be at [%.4f, %.4f, %.4f], target [%f, %f, %f]\n', ...
%             j, steps, i, position, waypointsPerStep(i,1:3),waypointsPerStep(i,10:12));
        if steps ~= startStep
            position = waypointsPerStep(i,1:3);
        end

        
        % check if misplanced
        if norm(position - waypointsPerStep(i,1:3)) > 0
            replan = true;
            stepToTarget = [0];
            droneID = i;

            currentTarget = waypointsPerStep(i,10:12);
            
            %   find the step to the next target for the current drone
            for x = (droneID + floor(startStep/dronesNum)*dronesNum):dronesNum:size(waypoints,1)
                if norm(waypoints(x,10:12) - currentTarget)~= 0
                    break;
                end
        
                stepToTarget(1) = stepToTarget(1) + 1;
            end
            
            drone = Drone(droneID, waypoints(droneID + floor(startStep/dronesNum)*dronesNum,1:3), currentTarget,waypoints(droneID + floor(startStep/dronesNum)*dronesNum,4:6),waypoints(droneID + floor(startStep/dronesNum)*dronesNum,7:9));
            
            drone.startPt = drone.position;
            replanDrones = [drone];

            figure(1);
            plot3(position(1), position(2), position(3),'.','MarkerSize',30,'Color', [1 0 0]);
            hold on;
        end
    end
    figure(1);
    plot3(waypointsPerStep(:,1), waypointsPerStep(:,2), waypointsPerStep(:,3),'.','MarkerSize',3,'Color', [0 1 0]);
    hold on;
end

while replan
    replan = false;
    
    potentialWayPts = [];
    for x = 1:length(replanSteps)
        replanSteps(x) = 0;
    end

    for x = 1:length(replanDrones)
        replanDrones(x).arrived = 0;
        replanDrones(x).position = waypoints(droneID + floor(startStep/dronesNum)*dronesNum,1:3);
        replanDrones(x).velocity = waypoints(droneID + floor(startStep/dronesNum)*dronesNum,4:6);
        replanDrones(x).acceleration = waypoints(droneID + floor(startStep/dronesNum)*dronesNum,7:9);
    end
    while(~all(arrivedDrones) == 1)
        if replan
            break;
        end

        for x = 1: length(replanDrones)
            checkPosition = [];
            if replanDrones(x).arrived
                continue;
            end
            waypointsPerStep = waypoints((floor(startStep/dronesNum) + replanSteps(x))*dronesNum + 1:(floor(startStep/dronesNum) + replanSteps(x))*dronesNum + dronesNum,:);
            
            for i = 1:dronesNum
                checkPosition = [checkPosition; waypointsPerStep(i,1:3), i, 0];
            end   
    
            checkPosition(drone.ID,:) = [];
    
    
            for i = 1:size(checkPosition,1)
                if norm(drone.position(:)-checkPosition(i,1:3)) <= 0.01
                    figure(1);
                    plot3(colDronesPerTime(i).position(1),colDronesPerTime(i).position(2),colDronesPerTime(i).position(3),'r.','MarkerSize',30);
                    disp('Collided');
                    replan = true;
                    %find the colliding drone and add it to the replan
                    %drone list
                    stepToTarget = [stepToTarget, 0];
                    for y = (replanDrones(x).ID + floor(startStep/dronesNum)*dronesNum):dronesNum:size(waypoints,1)
            
                        if norm(waypoints(i,10:12) - currentTarget)~= 0
                            break;
                        end
                
                        stepToTarget(end) = stepToTarget(end) + 1;
                    end
                    colDroneID = checkPosition(i,4);
                    colDrone = Drone(colDroneID, waypoints(colDroneID + floor(startStep/dronesNum)*dronesNum,1:3), currentTarget,waypoints(colDroneID + floor(startStep/dronesNum)*dronesNum,4:6),waypoints(colDroneID + floor(startStep/dronesNum)*dronesNum,7:9));
                    replanDrones = [replanDrones, colDrone];
                    arrivedDrones = [arrivedDrones, 0];
                end
            end
                   
            replanDrones(x) = apf.getNextStep(replanDrones(x),checkPosition);
            figure(1);
            plot3(replanDrones(x).position(1), replanDrones(x).position(2), replanDrones(x).position(3),'.','MarkerSize',10,'Color', [0 0 1]);
            hold on;
        
            % re-write waypoints
            waypointsPerStep(replanDrones(x).ID,:) = [replanDrones(x).position, replanDrones(x).velocity, replanDrones(x).acceleration, replanDrones(x).target];
       
            potentialWayPts = [potentialWayPts;waypointsPerStep];
    
            replanSteps(x) = replanSteps(x) + 1;
            if replanSteps(x) > stepToTarget(x)
                replan = true;
                foundNewTarget = false;
                for i = (replanDrones(x).ID + floor(startStep/dronesNum)*dronesNum) + stepToTarget(x)*dronesNum:dronesNum:size(waypoints,1)
                
                    if norm(waypoints(i,10:12) - replanDrones(x).target)~= 0
                        if foundNewTarget
                            break;
                        end
                        foundNewTarget = true;
                        replanDrones(x).target = waypoints(i,10:12);
                    end
                
                    stepToTarget(x) = stepToTarget(x) + 1;
                end
            end
    
                      
            if norm(replanDrones(x).position(:)-replanDrones(x).target(:)) <= 0.01
                replanDrones(x).arrived = true;
                disp("Re-planned and arrived!")
                fprintf("Origin Step %d, re-plan step %d, replanDrones(x) %d has arrived at target [%.2f, %.2f, %.2f] by replan\n", ...
                    stepToTarget(x), replanSteps(x), replanDrones(x).ID, replanDrones(x).target);
                arrivedDrones(x) = 1;
                replanDrones(x).arrived = 0;
                break;
            end
        end
    
    end
    
end
    

waypoints((floor(startStep/dronesNum)*dronesNum + 1):(floor(startStep/dronesNum)*dronesNum) + size(potentialWayPts,1),:) = potentialWayPts;
replanedDronesID = [];

for x = 1:length(replanDrones)
    for i = ((floor(startStep/dronesNum) + replanSteps(x))*dronesNum + replanDrones(x).ID):dronesNum:(floor(startStep/dronesNum) - 1 + stepToTarget(x))*dronesNum
        waypoints(i,1:3) = waypoints((floor(startStep/dronesNum) + replanSteps(x) - 1)*dronesNum + replanDrones(x).ID,1:3);
    end

    replanedDronesID = [replanedDronesID, replanDrones.ID];
end

prevStep = steps;
for j = floor(startStep/dronesNum)*dronesNum + 1:dronesNum:size(waypoints,1)

    waypointsPerStep = waypoints(j: j + dronesNum - 1,:);
    
    % set a checkNum to see if the corelated drone's target changing
    for checkNum = dronesNum:-1:1
        if ~ismember(checkNum,replanedDronesID)
            break;
        end
    end

    % if drone(checkNum)'s target changes, we see it meets the destination
    if steps == 17100
        pause(0.1);
    end
    if norm(waypointsPerStep(checkNum,10:12) - waypoints(j-1-dronesNum+checkNum,10:12)) ~= 0
        figure(1);
        plot3(waypointsPerStep(:,1), waypointsPerStep(:,2), waypointsPerStep(:,3),'.','MarkerSize',3,'Color', [0 1 0]);
        hold on;

        figure;
        plot3(waypointsPerStep(:,1), waypointsPerStep(:,2), waypointsPerStep(:,3),'.','MarkerSize',10,'Color', [0 0 0]);
        hold on;
    end
    drawWayPoints = [];
    for i = 1:size(waypointsPerStep,1)
        if ~ismember(i,replanedDronesID) || prevStep + replanSteps(find(replanedDronesID==i))*dronesNum < steps
            drawWayPoints = [drawWayPoints; waypointsPerStep(i,:)];
        end
    end
    figure(1);
    plot3(drawWayPoints(:,1), drawWayPoints(:,2), drawWayPoints(:,3),'.','MarkerSize',3,'Color', [0 1 0]);
    hold on;
    steps = steps + dronesNum;

    %fprintf('%d step %d, drone %d at [%f, %f, %f], target %d\n', j, steps, i, waypointsPerStep(i,1:3),waypointsPerStep(i,10));


end
