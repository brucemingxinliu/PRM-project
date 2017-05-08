function FLAG = CheckForCollision(node1,node2)
% FLAG = 1 if no collision
% FLAG = 0 if collision
global map;

res = 1;

mapSize = size(map,2);

tf2 = ispolycw(node1(:,1),node1(:,2));
if ~tf2
    [a,b] = poly2cw(node1(:,1),node1(:,2));
    node1 = [a,b];
end
tf3 = ispolycw(node2(:,1),node2(:,2));
if ~tf3
    [a,b] = poly2cw(node2(:,1),node2(:,2));
    node2 = [a,b];
end


for i = 1:mapSize-1
   obs = map{i};
   
    tf1 = ispolycw(obs(:,1),obs(:,2));
    if ~tf1
        [a,b] = poly2cw(obs(:,1),obs(:,2));
        obs = [a,b];
    end
   
   [x1,y1] = polybool('intersection',node1(:,1),node1(:,2),obs(:,1),obs(:,2));
   
   if isempty(x1)
       Flag = 1;
       
       [x2,y2] = polybool('intersection',node2(:,1),node2(:,2),obs(:,1),obs(:,2));
       
       if isempty(x2)
           Flag = 1;
       else
           Flag = 0;
       end
   
   else
       Flag = 0;
   end
   
   
end


end
