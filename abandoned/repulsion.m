function f_VRep = repulsion(drone,obstacles,affectDistance,etaR, etaV,target)
    f_VRep = [0, 0, 0];           %Initialize the force
    distToTarget = distanceCost(drone.position,target.position);
    n=2;    %n is an arbitrary real number which is greater than zero

    for i = 1 : size(obstacles,1)
        distToObst = distanceCost(drone.position,obstacles(i,:).position);
        
        %Drone is affecting by abstacle's repulsivefield 
        if distToObst <= affectDistance && distanceCost(drone.velocity, obstacles(i,:).velovity) > 0 %
            fRepByObst = etaR * (1/distToObst - 1/affectDistance) * distToTarget^n/distToObst^2 * differential(drone.position,obstacles(i,:).position)...
                + (n/2) * etaR * (1/distToObst - 1/affectDistance)^2 * distToTarget^(n-1) * differential(drone.position,target.position);

            fVByObst = etaV * distanceCost(obstacles(i,:).velovity, drone.velocity) * differential(drone.velocity,obstacles(i,:).velocity);
            f_VRep = f_VRep + fRepByObst + fVByObst;
        end
    end
end

