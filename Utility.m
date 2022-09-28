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

        function dis = getCos(self, vec1, vec2)
            dis = vec1 .* vec2 / (norm(vec1) * norm(vec2));
        end

        function coordinates = loadPtCld(filename)

            dimRow=true;
            
            rowCount = 1;
            
            vertexCount = [];
            
            fileID=fopen(filename);
            
            coordinates = [];
            
            x=0;
            y=0;
            z=0;
            
            while (~feof(fileID))
                currLine = textscan(fileID,'%s',1,'Delimiter','\n');
                currRow = char(currLine{1});
                splittedRow = strsplit(currRow,' ');
            
                if (~strcmp(splittedRow(1),'#') && ~strcmp(splittedRow(1),'OFF'))
            
                    splittedRow = str2double(splittedRow);
            
                    if(dimRow)
                        dimRow = false;
                        vertexCount = splittedRow(1);
                    else
                        if(rowCount <= vertexCount)
                            %vertexList{rowCount} = [splittedRow(1)*multiplier splittedRow(2)*multiplier splittedRow(3)*multiplier];
                            x = splittedRow(1);
                            y = splittedRow(2);
                            z = splittedRow(3);
                            coordinates = [coordinates, [rowCount,x,y,z]];
            
                        end
                        
                        rowCount = rowCount +1;
            
                        % progress
                        if(mod(rowCount,100)==0)
                            disp('.');
                        end
                    end
                end
            end
            
            fclose(fileID); %Close the input file
            
            %plot3(X,Y,Z, ".")
            disp("finished")
            end

    end
end

