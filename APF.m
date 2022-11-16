classdef APF
    %APF Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        
        %   Feild data sets
        attBound = 5;
        repDist1 = 1;
        repDist2 = 1.5;

        %   Coefficients
        epsilon = 5;
        etaR1 = 5;
        etaR2 = 200;
        
        util;
        
    end
    
    methods

        function self = APF()
            self.util = Utility();
        end


        % Compute the attractive force
        function f_att = attraction(self,drone,attBound,epsilon)
            dist = norm(drone.target - drone.position);

            %   To prevent attraction force grown too big when it's far from target
            %   Set an upper bound to the arraction force
            dist = min(dist, attBound);

            %   Return a the attraction force vector
            f_att = epsilon * (drone.target - drone.position) * dist/norm(drone.target - drone.position);
        end


        %   Calculate the total Velocity-Repulsive force
        function f_Rep = repulsion(self,drone,dronePositions,repDist1,repDist2,etaR1,etaR2)
            f_Rep = [0, 0, 0];           %Initialize the force
            blockingDroneNum = 0;
            for i = 1 : size(dronePositions,1)
                if isequal(drone.position,dronePositions(i,:))
                    continue;
                end

                distToObst = norm(drone.position-dronePositions(i,:));
                
                %Drone is affecting by abstacle's repulsivefield
                if distToObst <= repDist1
                    eta = etaR2;
                    blockingDroneNum = blockingDroneNum + 1;
                elseif distToObst <= repDist2 && distToObst > repDist1
                    eta = etaR1;
                    blockingDroneNum = blockingDroneNum + 1;
                else
                    eta = 0;
                end
                %   Calculate the repulsive force
                fRepByObst = eta * (1/distToObst - 1/repDist2) * (1/(distToObst^2)) * (-1) * self.util.differential(drone.position,dronePositions(i,:));
                    
                f_Rep = f_Rep + fRepByObst ;
            end

        end
       

        %Calculate the next step for current drone
        %   Consider add up kinematicConstrant later
        function [drone,targetExchange] = getNextStep(self,drone,dronePositions)

            targetExchange = 0;
            
            if size(dronePositions,1) ~= 0
                obstPositions = dronePositions(:,1:3);
            else
                obstPositions = [];
            end
            force = self.getTotalForce(drone,obstPositions);

            D_SR = norm(drone.position - drone.startPt);
            D_RE = norm(drone.position - drone.target);
            D_RO = inf;

            affectingDrone = [];

            closerObst = [];

            %   calculate is there obstacles between drone and target
            distToTarget = norm(drone.target - drone.position);

            for i = 1:size(dronePositions,1)

                obstToTarget = norm(drone.target - dronePositions(i,1:3));

                distToObst = norm(drone.position - dronePositions(i,1:3));

                if distToObst < 3.5 && dronePositions(i,5)
                    affectingDrone = [affectingDrone,dronePositions(i,4)];
                    if obstToTarget < distToTarget
                        closerObst = [i];
                    end
                end
                D_RO = min([D_RO,distToObst]);
                %fprintf("Dist to Obstacle [%.2f,%.2f,%.2f] = %.2f\n", dronePositions(i,:),D_RO)
            end

%             l = min([D_SR, D_RE, D_RO/2]) * drone.timeUnit;
            l = min([D_SR, D_RE]) * drone.timeUnit*5;
            if l == D_SR * drone.timeUnit*5
                md = "D_SR";
            elseif l == D_RE * drone.timeUnit*5
                md = "D_RE";
            else
                md = "D_RO";
            end

            if l < 0.5 * drone.accMax * drone.timeUnit^2
                l = 0.5 * drone.accMax * drone.timeUnit^2;
            elseif l > drone.vMax * drone.timeUnit 
                l = drone.vMax * drone.timeUnit;
            end
          
            distanceMove = l * force;
            
            drone.position = drone.position + distanceMove;
            drone.distTraveled = drone.distTraveled + norm(distanceMove);
            lastV = drone.velocity;
            drone.velocity = ((2 * norm(distanceMove) / drone.timeUnit) - norm(drone.velocity)) * force;
            if norm(drone.velocity) >= 3
                drone.velocity = drone.velocity * 3/norm(drone.velocity);
            end

            drone.acceleration = (drone.velocity - lastV)/drone.timeUnit;
  
%             if D_RO <= 2.55 
%                 disp(dronePositions);
%             end
            fprintf("Drone %d at position [%.2f,%.2f,%.2f], targeting [%.2f,%.2f,%.2f] moving %.4f based on %s with speed %.4f, with %.4f left, dist to obstacle %.4f\n", ...
               drone.ID, drone.position,drone.target, l, md, norm(drone.velocity), D_RE, D_RO);

            newDroneToTarget = norm(drone.target - drone.position);
            
            if newDroneToTarget > distToTarget && (length(affectingDrone)>=2) && (length(closerObst)>=1)
                targetExchange = closerObst(1);
                drone.targetExchangeCounter = drone.targetExchangeCounter+1;
            end
            

        end
            
        %   Calculate the total force of the field on the drone
        function f_total = getTotalForce(self,drone,dronePositions)
            f_att = self.attraction(drone,self.attBound,self.epsilon);
            f_rep = self.repulsion(drone,dronePositions,self.repDist1,self.repDist2,self.etaR1,self.etaR2);
           
            f_total = f_att + f_rep;


            %fprintf('total force [%f,%f,%f] \n', f_total);
            %f_total = self.util.getUnitVec(f_total);
            %f_attMax = self.epsilon * norm(drone.target - drone.startPt);
            %f_total = self.util.getNormalized(f_attMax, f_total);
            f_total = f_total/norm(f_total);

        end

    end
end

