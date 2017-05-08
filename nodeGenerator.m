function nodeGenerator(NumNodes)

global qstart qend states;


states = zeros(NumNodes, 2);

xnew =  (10*rand(1));
ynew =  (10*rand(1));

qnew = [xnew ynew]; 



col = CheckForCollision(node1,node2);
col =1;
while (col)~=1
    xnew =  (10*rand(1));
    ynew =  (10*rand(1));

    qnew = [xnew ynew]; 

    plot(xnew, ynew, '.');
    hold on;

    col = checkForCollision(node1,node2);    
end
states(1,:) = qnew;


totStat = 1;
while totStat < NumNodes
    qcurr = states(totStat,:);

    
%generate new state

xnew = qcurr(1) + (-NumNodesStep + 2*NumNodesStep*rand(1));
ynew = qcurr(2) + (-NumNodesStep + 2*NumNodesStep*rand(1));
O1new = qcurr(3) + (-pi + 2*pi*rand(1));
O2new = qcurr(4) + (-pi + 2*pi*rand(1));
xnew =  (2*rand(1));
ynew =  (3*rand(1));


col =1;

%col = checkForCollision(body1,body2);

if (col) == 1
plot(xnew, ynew, '.');

end
    
end

plot[node1,node2];
states(end,:) = qend;


% create resulting states
newStates = zeros(NumNodes+2,4);
newStates(1,:) = qstart;
newStates(2:(end-1),:) = states(:,:);
newStates(end,:) = qend;
states = newStates;

end
