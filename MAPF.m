classdef MAPF
    %APF Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        
        drones
    
        %   Feild data sets
        timeUnit = 1/25;
        stepsize = 0.2;
        attBound = 5;
        repDist = 1;
        threshold = 0.2;

        %   Coefficients
        epsilon = 0.5;
        etaR = 0.2;
        etaV = 0.2;
        
        util;
        
    end
    
    methods

        function self = MAPF(drones)
            self.drones = drones;
            self.path = startPt;
       
            self.util = Utility();
            %self.fAttMax = norm(self.attraction(startPt,target,self.attBound,self.epsilon));
        end


        % Compute the attractive force
        function f_att = attraction(self,dronePos,target,distBound,epsilon)
            dis = norm(dronePos-target);
        
            %   To prevent attraction force grown too big when it's far from target
            %   Set an upper bound to the arraction force
            if dis <= distBound
                fx = epsilon * (target(1) - dronePos(1));
                fy = epsilon * (target(2) - dronePos(2));
                fz = epsilon * (target(3) - dronePos(3));
            else
                fx = distBound * epsilon * (target(1) - dronePos(1)) / dis;
                fy = distBound * epsilon * (target(2) - dronePos(2)) / dis;
                fz = distBound * epsilon * (target(3) - dronePos(3)) / dis;
            end
        
            %   Return a the attraction force vector
            f_att = [fx, fy, fz];
        end


        %   Calculate the total Velocity-Repulsive force
        function f_VRep = repulsion(self,drone,obstacles,affectDistance,etaR, etaV,target)
            f_VRep = [0, 0, 0];           %Initialize the force
            distToTarget = norm(drone.position - target);
            n=2;    %n is an arbitrary real number which is greater than zero
        
            for i = 1 : size(obstacles,2)
                % skip the drone itself
                if isequal(drone.position,obstacles(i).position) 
                    continue;
                end

                distToObst = norm(drone.position-obstacles(i).position);
                
                %Drone is affecting by abstacle's repulsivefield
                if distToObst <= affectDistance ...
                    && self.util.getCos(norm(drone.velocity - obstacles(i).velocity), norm(drone.position - obstacles(i).position)) > 0
                    %   Calculate the repulsive force
                    a = self.util.differential(drone.position,obstacles(i).position);
                    b = self.util.differential(drone.position,target);
                    fRepByObst = etaR * (1/distToObst - 1/affectDistance) * distToTarget^n/distToObst^2 * -1 * self.util.differential(drone.position,obstacles(i).position)...
                        + (n/2) * etaR * (1/distToObst - 1/affectDistance)^2 * distToTarget^(n-1) * -1 * self.util.differential(drone.position,target);
                    
                    %   Calculate the velocity repulsive force
                    fVByObst = etaV * norm(obstacles(i).velocity - drone.velocity) * self.util.differential(obstacles(i).position, drone.position);
                    
                    f_VRep = f_VRep + fRepByObst ;%+ fVByObst;
                    %fprintf('\naffect by [%d,%d,%d], with rep of [%f,%f,%f] and Vrep of [%f,%f,%f]\n',obstacles(i).position, fRepByObst,fVByObst);
                end
            end

        end
       

        %Calculate the next step for current drone
        %   Consider add up kinematicConstrant later
        function [nextPos,actualV] = getNextStep(self,drone)
            force = self.getTotalForce(drone);
            %   The maximum speed the drone can reach is corelated to force, but was constarined by vMAx
            targetV = drone.vMax * force;
            
            targetVValue = norm(targetV);
            if targetVValue > drone.vMax
                targetV = targetV * (drone.vMax/targetVValue);
                targetVValue = drone.vMax;
            end

            accTime = min(abs(targetVValue - norm(drone.velocity))/drone.accMax, self.timeUnit);

            % check if the drone is speeding up or slowing down
            if targetVValue < norm(drone.velocity)
                flag = -1;
            else
                flag = 1;
            end

            actualVValue = norm(drone.velocity) + drone.accMax * accTime * flag;


            actualV = (actualVValue/targetVValue) * targetV;

                        
            nextPos = drone.position + 0.5 * (actualV + drone.velocity) * accTime + actualV * (self.timeUnit-accTime);
            fprintf('next Pos [%f,%f,%f] with speed %f \n', nextPos, actualVValue);
        end
            
        %   Calculate the total force of the field on the drone
        function f_total = getTotalForce(self,drone)
            f_att = self.attraction(drone.position,self.target,self.attBound,self.epsilon);
            f_rep = self.repulsion(drone,self.obstacles,self.repDist,self.etaR, self.etaV,self.target);
           

            f_total = f_att + f_rep;


            fprintf('total force [%f,%f,%f] \n', f_total);
            %f_total = self.util.getUnitVec(f_total);

            f_total = self.util.getNormalized(self.fAttMax, f_total);

        end

        function saveCSV(self,folder)
            writematrix(self.path, ['./',folder,'/pathMatrix'],'Delimiter',',');
            writematrix()
        end

    end
end

