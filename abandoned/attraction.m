function f_att = attraction(dronePosition,target,distBound,epsilon_att)
    dis = distanceCost(dronePosition,target);

    %To prevent attraction force grown too big when it's far from target
    %Set an upper bound to the arraction force

    if dis <= distBound
        fx = epsilon_att * (target(1) - dronePosition(1));
        fy = epsilon_att * (target(2) - dronePosition(2));
        fz = epsilon_att * (target(3) - dronePosition(3));
    else
        fx = distBound * epsilon_att * (target(1) - dronePosition(1)) / dis;
        fy = distBound * epsilon_att * (target(2) - dronePosition(2)) / dis;
        fz = distBound * epsilon_att * (target(3) - dronePosition(3)) / dis;
    end

    %Return a the attraction force vector
    f_att = [fx, fy, fz];
end

