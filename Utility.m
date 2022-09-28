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
            output = (pos2 - pos1)/norm(pos1-pos2);
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

