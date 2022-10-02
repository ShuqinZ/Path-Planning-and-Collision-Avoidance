clc; clear; close all;
util = Utility();
startPtFile = "Point Cloud Squence/pt1379_change.ptcld";
targetPtFiles = ["pt1547_change.ptcld", ""];

stoptime = 1;
removeWhenCollide = 0;
iterations = 1;
targets = [];
direction = [];
distLeft = [];
timeunit = 1/100;

startPts = util.loadPtCld(startPtFile);

for i = 1:size(targetPtFiles)
    targets(i) = util.loadPtCld("Point Cloud Squence/" + targetPtFiles(i));
end

for i = 1:size(startPts)
    drones(i) = Drone(i,startPts(i,:),targets(i,:),[0,0,0],[0,0,0]);
end

waypointsPerStep = startPts;

for k = 1:iterations
    for j = 1:size(targets)
        target = targets(i);
        step = 0;

        % set the moving direction of each drones
        for i = 1:size(target)
            direction(i) = util.differential(drones(i).position, target(i));
            distLeft(i) = norm(drones(i).position - target(i));
        end
        
        while arriveNum ~= size(drones)
            step = step + 1;
            
            for i = 1:size(drones)
                accTime = 0;


                % Collision detection
                for m = 1:i
                    if all(abs(drones(i).position(:)- waypointsPerStep(i)))<=[0.0001,0.0001,0.0001]
                        if removeWhenCollide

                        end
                        %记录并显示碰撞点
                    end
                end

                % if the drone is already arrived, or just arrived, skip
                if drones(i).arrived || all(abs(drones(i).position - drones(i).target)<=[0.0001,0.0001,0.0001])
                    drones(i).arrived = 1;
                    continue
                end
                % Check if the drone needs to speed up or slow down
                %   If the distance left is no bigger than the minimum
                %   slowing down distance
                if distLeft <= 0.5 * vMax * (drones(i).vMax/drones(i).accMax)
                    flag = -1;
                    accTime = Timeunit;
                elseif  norm(drones(i).velocity) == drones(i).vMax
                    flag = 1;
                    accTime = 0;
                else
                    flag = 1;
                    accTime = (drones(i).vMax - norm(drones(i).velocity))/drones(i).accMax; 

                end

                newV = drones(i).velocity + flag * drones(i).accMax * accTime * direction(i);
                positionMoved = 0.5 * (drones(i).velocity + newV) * accTime + newV * (timeunit-accTime);
                distMoved = norm(postitionMoved);
                distLeft(i) = distLeft(i) - distMoved;

                drones(i).position = drones(i).position + positionMoved;
                drones(i).velocity = newV;

                waypointsPerStep(i) = drones(i).position;

            end
        end

        pause(stoptime);
    end
end

