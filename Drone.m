classdef Drone
   properties  
    ID;
    timeUnit = 1/25;
    %   Velovity Restrict (use data frome Dji)
    yawPerStep;
    pitchPerStep;
    yawMax;
    climbAngleMax;
    subAngleMax;
    vMax = 3;
    accMax = 3;

    distTraveled;

    %   State Info
    velocity;
    acceleration;
    position;
    target;
    startPt;
    waypoints;
    arrived;
    removed;
    goDark;

    targetExchangeCounter;
    
   end

   methods
       function self=Drone(ID, position, target, initV, initA)
           self.ID = ID;

           %   Velovity Restrict (use data frome Dji)
           self.yawPerStep = 250/180 * pi * self.timeUnit;
           self.pitchPerStep = 250/180 * pi * self.timeUnit;
           self.yawMax = 25/180 * pi * self.timeUnit;
           self.climbAngleMax = 35/180 * pi;
           self.subAngleMax = 35/180 * pi;

           self.target = target;
        
           %   State Info
           self.velocity = initV;
           self.acceleration = initA;
           self.position = position;
           self.distTraveled = 0;
           self.startPt = [];
           self.waypoints = position;

           self.arrived = false;
           self.removed = false;
           self.targetExchangeCounter = 0;
           self.goDark = false;
       end

   end
end