clc; clear; close all;
util = Utility();
startPtFile = "Point Cloud Squence/pt1379_change.ptcld";
targetPtFile = "Point Cloud Squence/pt1547_change.ptcld";

startPts = util.loadPtCld(startPtFile);
targets = util.loadPtCld(targetPtFile);

drones = [];

%   initialize all the drones
for i = 1:size(startPts)
    drones = [drones, Drone(i,startPts(i),targets(i),[0,0,0],[0,0,0])];
end


apf = MAPF(drones);

waypoints = [];
steps = 0;

while steps < 100
    for i = 1:size(drones)
    %while ~all(abs(drone.position(:)-target(:))<=[0.00001,0.00001,0.00001])
        a = abs(drones(i).position(:)- drones(i).target(:));
        [drones(i).position, drones(i).velocity] = apf.getNextStep(drones(i),drones);
        waypoints = [waypoints; drones(i).position];

        %disp(steps);
    end

    steps = steps + 1;
end
plot3(targets(:,1),targets(:,2),targets(:,3),'r.','MarkerSize',30);
hold on;
plot3(obstacles(:,1),obstacles(:,2),obstacles(:,3),'g.','MarkerSize',30);
hold on;
plot3(waypoints(:,1),waypoints(:,2),waypoints(:,3),'b.','MarkerSize',10);
disp(waypoints);

