function res = checkForCollision(body1,body2)
% res = 1 if no collision
% res = 0 if collision
global map;

res = 1;

mapSize = size(map,2);


% check for clockwise polygon
tf2 = ispolycw(body1(:,1),body1(:,2));
if ~tf2
    [a,b] = poly2cw(body1(:,1),body1(:,2));
    body1 = [a,b];
end
tf3 = ispolycw(body2(:,1),body2(:,2));
if ~tf3
    [a,b] = poly2cw(body2(:,1),body2(:,2));
    body2 = [a,b];
end


for i = 1:mapSize-1
   obs = map{i};
   
   % check clockwise obstacle
    tf1 = ispolycw(obs(:,1),obs(:,2));
    if ~tf1
        [a,b] = poly2cw(obs(:,1),obs(:,2));
        obs = [a,b];
    end
   
   [x1,y1] = polybool('intersection',body1(:,1),body1(:,2),obs(:,1),obs(:,2));
   
   if isempty(x1)
       res = res*1;
       
       [x2,y2] = polybool('intersection',body2(:,1),body2(:,2),obs(:,1),obs(:,2));
       
       if isempty(x2)
           res = res*1;
       else
           res = res*0;
       end
   
   else
       res = res*0;
   end
   
   
end


end