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
    vMax = 19;
    accMax = 14;

    distMoved;

    %   State Info
    velocity;
    acceleration;
    position;
    
   end

   methods
       function self=Drone(ID, position, initV, initA)
           self.ID = ID;

           %   Velovity Restrict (use data frome Dji)
           self.yawPerStep = 250/180 * pi * self.timeUnit;
           self.pitchPerStep = 250/180 * pi * self.timeUnit;
           self.yawMax = 25/180 * pi * self.timeUnit;
           self.climbAngleMax = 35/180 * pi;
           self.subAngleMax = 35/180 * pi;
        
           %   State Info
           self.velocity = initV;
           self.acceleration = initA;
           self.position = position;
           self.distMoved = 0;
       end

   end
end