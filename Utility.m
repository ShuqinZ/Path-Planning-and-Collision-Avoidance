classdef Utility
    %UTILITY Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        
    end

    methods


        % calculate distance between two vector
        function h=distanceCost(self,a, b)         
	        h = sqrt(sum((a-b).^2));
        end
              
        %   calculate the differential of two vector
        function output = differential(self,pos1,pos2)
            output_x = (pos1(1) - pos2(1)) / self.distanceCost(pos1,pos2);
            output_y = (pos1(2) - pos2(2)) / self.distanceCost(pos1,pos2);
            output_z = (pos1(3) - pos2(3)) / self.distanceCost(pos1,pos2);
            output = [output_x,output_y,output_z];
        end
        

        %   normalize the vector
        function unitVec = getUnitVec(self,vec)
            unitVec = vec/sqrt(sum(vec.^2));
        end

        function normVec = getNormalized(self, distMax, vec)
                normVec = vec / distMax;

        end

    end
end

