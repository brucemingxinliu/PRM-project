function roadMap = roadMap(kClose)

global states;

stateSize = size(states,1);

roadMap = [];

for i = 1:stateSize
    % start with the starting state
    
    qcurr = states(i,:);
    pts1 = qcurr(1:2);
    visibleNodes = [];
    possLines = [];
    
    % find the closest k neigboors to current state
    for j =1:stateSize
        if j~= i
            qnext = states(j,:);
            pts2 = qnext(1:2);
            % check visible nodes 
            res = checkConvexHull(qcurr,qnext);
            if res 
               visibleNodes = [visibleNodes;qnext];
               possLines = [possLines; [pts1 pts2] ];
            end            
        end
    end
    
    sizeVis = size(visibleNodes,1);
    
    if sizeVis > 0
       
       visibleNewNodes = [];
       possNewLines = [];
       % first check for already existing lines
       for k = 1:sizeVis
           qnow = visibleNodes(k,:);
           % check for unidirection
           lnow = possLines(k,:);
           linvert = [lnow(3:4) lnow(1:2)];
           if not(findSame(roadMap,lnow)) && not(findSame(roadMap,linvert))
               visibleNewNodes = [visibleNewNodes;qnow];
               possNewLines = [possNewLines;lnow];
           end          
       end
       % now find the closest nodes       
       sizeNewNodes = size(visibleNewNodes,1);
       distanceArray = zeros(sizeNewNodes,1);
       for k = 1:sizeNewNodes
           qnow = visibleNewNodes(k,:);
           pts2 = qnow(1:2);          
           distanceArray(k) = findSqDistance(pts1,pts2);           
       end
       % sort the distances acc. to Euclidian distance
       [val,ind] = sort(distanceArray);
       
       sizeDist = size(distanceArray,1);
       % now select the closest k neighboor nodes
       if sizeDist > kClose
           for k = 1:kClose
               q2Sel = visibleNewNodes(ind(k),:);
               pts2 = q2Sel(1:2);
               roadMap = [roadMap; [pts1 pts2]];
           end           
       else
           for k = 1:sizeDist
                q2Sel = visibleNewNodes(ind(k),:);
                pts2 = q2Sel(1:2);
                roadMap = [roadMap; [pts1 pts2]];
           end          
       end
       
       
    end

    
end

end