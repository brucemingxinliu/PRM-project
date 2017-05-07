MapCreator;
hold on;
NumNodes = 5000;
%num of neighnors to check
k = 2;


qstart = [2 4];

qend = [9 4.7];
start = qstart(1:2);
goal = qend(1:2);
tic

nodeGenerator_TEST(NumNodes);

for i = 1:NumNodes
    
    plot(xnew(i), ynew(i));
    hold on;
    
end
%% plot the path
%tic
%roadMap = roadMap(k);
%for i = 1:size(roadMap,1) 
    %plot([roadMap(i,1) roadMap(i,3)],[roadMap(i,2) roadMap(i,4)],'k');
    %hold on;
%end
%toc 

%tic
%roadMapSol = aStarSearch(roadMap);

%if not(isempty(roadMapSol))
  %  for i = 1:size(roadMapSol,1) 
      %  plot([roadMapSol(i,1) roadMapSol(i,3)],[roadMapSol(i,2) roadMapSol(i,4)],'g','LineWidth',3);
      %  hold on;
    %end  
%end

toc