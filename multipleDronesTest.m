clc; clear; close all;
util = Utility();
startPtFile = "";
targetPtFile = "";

startPts = util.loadPtCld(startPtFile);
targets = util.loadPtCld(targetPtFile);

drones = [];

%   initialize all the drones
for i = 1:size(startPts)
    drones = [drones, Drone(i,startPt(i),targets(i),[0,0,0],[0,0,0])];
end


apf = MAPF(drones);

waypoints = [];
steps = 0;


while steps < 100
%while ~all(abs(drone.position(:)-target(:))<=[0.00001,0.00001,0.00001])
    a = abs(drone.position(:)-target(:));
    [drone.position, drone.velocity] = apf.getNextStep(drone);
    waypoints = [waypoints; drone.position];
    steps = steps + 1;
    %disp(steps);
end
plot3([0,10],[0,10],[0,10],'r.','MarkerSize',30);
hold on;
plot3(obstacles(:,1),obstacles(:,2),obstacles(:,3),'g.','MarkerSize',30);
hold on;
plot3(waypoints(:,1),waypoints(:,2),waypoints(:,3),'b.','MarkerSize',10);
disp(waypoints);

