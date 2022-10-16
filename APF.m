classdef APF
    %APF Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        
        %   Feild data sets
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

        function self = APF()
            self.util = Utility();
        end


        % Compute the attractive force
        function f_att = attraction(self,drone,distBound,epsilon)
            dis = norm(drone.position-drone.target);
        
            %   To prevent attraction force grown too big when it's far from target
            %   Set an upper bound to the arraction force
            fx = epsilon * (drone.target(1) - drone.position(1));
            fy = epsilon * (drone.target(2) - drone.position(2));
            fz = epsilon * (drone.target(3) - drone.position(3));
            %   Return a the attraction force vector
            f_att = [fx, fy, fz];
        end


        %   Calculate the total Velocity-Repulsive force
        function f_VRep = repulsion(self,drone,dronePositions,affectDistance,etaR)
            f_VRep = [0, 0, 0];           %Initialize the force
            n=2;    %n is an arbitrary real number which is greater than zero

            for i = 1 : size(dronePositions,1)
                if isequal(drone.position,dronePositions(i,:))
                    continue;
                end

                distToObst = norm(drone.position-dronePositions(i,:));
                
                %Drone is affecting by abstacle's repulsivefield
                if distToObst <= affectDistance
                    %   Calculate the repulsive force
                    fRepByObst = etaR * (1/distToObst - 1/affectDistance) * (1/distToObst^2) * (-1) * self.util.differential(drone.position,dronePositions(i,:));
                    
                    f_VRep = f_VRep + fRepByObst ;
                end
            end

        end
       

        %Calculate the next step for current drone
        %   Consider add up kinematicConstrant later
        function drone = getNextStep(self,drone,dronePositions)
            force = self.getTotalForce(drone,dronePositions);

            D_SR = norm(drone.position - drone.startPt);
            D_RE = norm(drone.position - drone.target);
            D_RO = inf;
            for i = 1:size(dronePositions)
                D_RO = min([D_RO,norm(drone.position - dronePositions(i,:))]);
                fprintf("D_RO = %.4f", D_RO)
            end

            l = min([D_SR, D_RE, D_RO/2]) * drone.timeUnit;
            if l < 0.5 * drone.accMax * drone.timeUnit^2
                l = 0.5 * drone.accMax * drone.timeUnit^2;
            elseif l > drone.vMax * drone.timeUnit
                l = drone.vMax * drone.timeUnit;
            end
            distanceMove = l * force;
            
            drone.position = drone.position + distanceMove;
            drone.distTraveled = drone.distTraveled + norm(distanceMove);

            drone.velocity = ((2 * norm(distanceMove) / drone.timeUnit) - norm(drone.velocity)) * force;
            if l == D_SR * drone.timeUnit
                md = "D_SR";
            elseif l == D_RE * drone.timeUnit || l == 0.5 * drone.accMax * drone.timeUnit^2
                md = "D_RE";
            else
                md = "D_RO";
            end
            fprintf("Drone %d at position [%.2f,%f.2,%.2f],moving %.4f based on %s with speed %.4f, with %.4f left\n", drone.ID, drone.position, l, md, norm(drone.velocity), D_RE);

        end
            
        %   Calculate the total force of the field on the drone
        function f_total = getTotalForce(self,drone,dronePositions)
            f_att = self.attraction(drone,self.attBound,self.epsilon);
            f_rep = self.repulsion(drone,dronePositions,self.repDist,self.etaR);
           
            f_total = f_att + f_rep;


            %fprintf('total force [%f,%f,%f] \n', f_total);
            %f_total = self.util.getUnitVec(f_total);
            %f_attMax = self.epsilon * norm(drone.target - drone.startPt);
            %f_total = self.util.getNormalized(f_attMax, f_total);
            f_total = f_total/norm(f_total);

        end

    end
end

