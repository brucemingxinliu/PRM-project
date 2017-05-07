PlannerGraph.m


classdef PlannerGraph < handle & matlab.mixin.Copyable
    %This class is for internal use only. It may be removed in the future.
    
    %PlannerGraph Graph class
    %   G = PlannerGraph(DIM, METRIC) constructs a graph with given
    %   dimension DIM. DIM must be a finite value that is greater or equal
    %   to 2. METRIC can be 'euclidean' or 'se2'.
    %
    %   The purpose of this class is to create a graph and perform
    %   computation over the graph. This class provides support for undirected
    %   graph that are embedded in coordinate system. The graph also has
    %   symmetric weight edges (A to B is same weight as B to A) and no loops
    %   (edges from A to A). Nodes and edges are represented by labels.
    %
    %   PlannerGraph methods:
    %       addNode              - Add a node to the graph
    %       addEdge              - Add an edge(s) to the graph
    %       setNodeData          - Set data for a given node
    %       getNodeData          - Get data from a given node
    %       nodeCoordinate       - Find coordinate of given node label
    %       edgesFromNode        - Find edges given node
    %       edgeWeight           - Get weight of given edge label
    %       setEdgeWeight        - Set weight of given edge
    %       componentFromNodes   - Label of component of a given node
    %       closestNode          - Find closest node from given coordinate
    %       distanceFromAllNodes - Distances from a given coordinate to all nodes
    %       aStar                - Perform A* search algorithm
    %
    %   PlannerGraph properties:
    %       NumNodes        - (Read-Only) Number of nodes in the graph
    %       NumEdges        - (Read-Only) Number of edges in the graph
    %       NumComponents   - (Read-Only) Number of components in the graph
    %       EdgeList        - (Read-Only) List of node label pairs that correspond to each edges.
    %       NodeList        - (Read-Only) List of coordinates of every nodes in the graph
    %       Dimension       - (Read-Only) Dimension of graph
    %
    %   Example:
    %
    %       % Create a graph
    %       g = robotics.algs.internal.PlannerGraph(2, 'euclidean');
    %
    %       % Add nodes
    %       addNode(obj, [1, 1]);
    %       addNode(obj, [2, 2]);
    %
    %       % Add an edge
    %       addEdge(1, 2);
    
    %   Copyright 2014-2015 The MathWorks, Inc.
    
    %   Copyright (C) 1993-2014, by Peter I. Corke
    %
    %   This file is part of The Robotics Toolbox for Matlab (RTB).
    %
    %   http://www.petercorke.com
    %
    %   Peter Corke 8/2009.
    
    properties (SetAccess = private)
        %Dimension - Dimension of graph.
        Dimension
    end
    properties (Access = ?matlab.unittest.TestCase)
        %AdjacencyMatrix Adjacency matrix for the graph
        %   The row and column index refers to the respective node label
        %   and the value in the matrix refers to the weight of the edge
        %   connecting those two nodes.
        AdjacencyMatrix
    end
    
    properties (Access = private)
        %Nodes List of coordinates of every nodes in the graph.
        %   Nodes is a list of coordinates of pre-defined size. The nodes
        %   that are not added yet are represented as NaN values. Each
        %   column represents the coordinate of corresponding node label.
        %   For example, if the node label is N, the coordinate C of the node
        %   of graph G is C = G.Nodes(:,N).
        Nodes
        
        %CurrentLabel The latest node label of a node that was added
        CurrentLabel
        
        %NodeLabels List of all node labels.
        NodeLabels
        
        %NodeLabelSet Set of all labels corresponding to components
        NodeLabelSet
        
        %NodeData A cell array of data in every nodes.
        NodeData
        
        %IsEuclidean name A boolean representing whether the metric is Euclidean
        IsEuclidean
    end
    
    properties (Dependent, SetAccess = private)
        %NumNodes Number of nodes in the graph.
        NumNodes;
        %NumEdges Number of edges in the graph.
        NumEdges;
        
        %EdgeList List of node label pairs that correspond to each edges.
        %   Each column represents node label pairs of corresponding edge
        %   label. For example, if edge label is E, the node label pair  NL
        %   of graph G is NL = G.EdgeList(:,E).
        EdgeList
        
        %NodeList List of coordinates of every nodes in the graph.
        %   Each column represents the coordinate of corresponding node label.
        %   For example, if the node label is N, the coordinate C of the node
        %   of graph G is C = G.NodeList(:,N).
        NodeList
    end
    properties (SetAccess = private)
        %NumComponents Number of components in the graph.
        NumComponents
    end
    methods
        %%
        function obj = PlannerGraph(dimension, metric, numnodes)
            %PlannerGraph Create an empty graph
            %   Please see the class documentation
            %   (help robotics.algs.internal.PlannerGraph) for more details.
            
            if strcmpi(metric, 'euclidean')
                isEuclidean = true;
            else
                isEuclidean = false;
            end
            
            if nargin < 3
                numnodes = 100;
            end
            
            % Assign properties
            obj.Nodes = nan(dimension, numnodes);             % no node in the list
            obj.AdjacencyMatrix = zeros(numnodes, numnodes);  % no edges because there are no nodes
            
            obj.Dimension = dimension;
            obj.NodeLabelSet = zeros(1, 0);         % Empty set of label
            obj.NodeLabels = zeros(1, 0);           % no label
            obj.CurrentLabel = 0;                   % current node is 0
            obj.NumComponents = 0;                  % no disconnected component
            obj.IsEuclidean = isEuclidean;
            obj.NodeData = [];                      % no data since there is no node
        end
        
        function n = get.NumNodes(obj)
            n = size(obj.NodeList,2);               % number of columns
        end
        
        function ne = get.NumEdges(obj)
            ne = size(obj.EdgeList,2);              % number of columns
        end
        
        function edgeList = get.EdgeList(obj)
            % As it is undirected graph, only using lower triangular part
            % of the adjacency matrix
            [r, c] = find(tril(obj.AdjacencyMatrix));
            edgeList = [r, c]';
        end
        
        
        function nodeList = get.NodeList(obj)
            nodeList = obj.Nodes(:, ~isnan(obj.Nodes(1,:)));
        end
        
        function label = addNode(obj, coordinate)
            %addNode Add a node to the graph
            %   L = addNode(G,C) adds a node/Node with coordinate C to
            %   graph G and returns the node label L.
            %
            %   See also addEdge.
            
            % Given coordinate has to have the same dimension of the graph
            
            nodeSameXCoordinate = abs(obj.NodeList(1,:)-coordinate(1))<eps;
            nodeSameYCoordinate = abs(obj.NodeList(2,:)-coordinate(2))<eps;
            
            sameNode = nodeSameXCoordinate & nodeSameYCoordinate;
            
            if any(sameNode)
                label = find(sameNode,1);
            else
                % append the coordinate as a column in the Node List
                obj.Nodes(:,obj.CurrentLabel+1) = coordinate(:);
                label = obj.CurrentLabel+1;
                obj.NodeLabels(label) = obj.newLabel();
            end
        end
        
        function addEdge(obj, node1, node2, w)
            %addEdge Add an edge(s) to the graph
            %   addEdge(G, N1, N2) adds an edge from node label N1 to node
            %   label N2. The edge weight is the distance between the nodes.
            %   N1 and N2 are scalar numbers representing nodes.
            %
            %   addEdge(G, N1, N2, W) adds an edge from node label N1 to node
            %   label N2 with weight W.
            %
            %   See also addNode.
            
            % node1 is a positive scalar integer
            % node2 is a positive scalar integer
            
            % Add weight to the adjacency matrix
            if nargin < 4
                w = obj.distanceBetweenNodes(node1,node2);
            end
            
            obj.AdjacencyMatrix(node1, node2) = w;
            obj.AdjacencyMatrix(node2, node1) = w;
            
            % update labels of nodes
            if obj.NodeLabels(node2) ~= obj.NodeLabels(node1)
                
                % combine two labels
                obj.merge(obj.NodeLabels(node2), obj.NodeLabels(node1));
            end
        end
        
        function data = setNodeData(obj, node, data)
            %setNodeData Set data for a given node
            %   D = setNodeData(G, N, D) sets the data of Node N to D
            %   which can be of any type such as number, struct, object
            %   or cell array.
            %
            %   See also getNodeData.
            
            obj.NodeData(node) = data;
        end
        
        function data = getNodeData(obj, node)
            %getNodeData Get data from a given node
            %   D = getNodeData(G, N) gets the data of Node N
            %   which can be of any type such as number, struct, object or
            %   cell array.
            %
            % See also setNodeData.
            
            data = obj.NodeData(node);
        end
        
        function coordinate = nodeCoordinate(obj, node)
            %nodeCoordinate Find coordinate of given node label
            %   C = nodeCoordinate(G, N) is the coordinate of corresponding
            %   node label N
            %
            %   See also edgesFromNode.
            
            coordinate = obj.Nodes(:,node);
        end
        
        function edges = edgesFromNode(obj, node)
            %edgesFromNode Find edges given node
            %   E = edgesFromNode(G, N) is a vector containing the label of
            %   all edges from node label N.
            %
            %   See also nodeCoordinate.
            
            edges = [find(obj.EdgeList(1,:) == node) find(obj.EdgeList(2,:) == node)];
        end
        
        function weight = edgeWeight(obj, edge)
            %edgeWeight Get weight of given edge label
            %   W = edgeWeight(G, E) get list of weight W from list of edge
            %   label E.
            %
            %   See also setEdgeWeight.
            
            edgeNodes = obj.EdgeList(:,edge);
            weight = obj.AdjacencyMatrix(edgeNodes(1), edgeNodes(2));
        end
        
        function weight = setEdgeWeight(obj, edge, weight)
            %setEdgeWeight Set weight of given edge label
            %   setEdgeWeight(G, E, W) set weight of edge label E to W.
            %
            %   See also edgeWeight.
            
            edgeNodes = obj.EdgeList(:,edge);
            
            obj.AdjacencyMatrix(edgeNodes(1), edgeNodes(2)) = weight;
            obj.AdjacencyMatrix(edgeNodes(2), edgeNodes(1)) = weight;
        end
        
        function componentList = componentFromNodes(obj, nodes)
            %componentFromNodes Label of component of a given list of nodes
            %   C = componentFromNodes(G, N) returns a vector of component
            %   labels for the input nodes N. Each label represents the
            %   graph component in which the corresponding node resides.
            
            componentList = [];
            
            for i=1:numel(nodes)
                logicalArray = ismember(obj.NodeLabelSet, obj.NodeLabels(nodes(i)));
                componentList = [componentList find(logicalArray)]; %#ok<*AGROW>
            end
        end
        
        function node = closestNode(obj, coordinate)
            %closestNode Find closest node from given coordinate
            %   N = closest(G, C) is the node geometrically closest to
            %   coordinate C.
            %
            %   See also distanceFromAllNodes.
            
            % check whether the graph is empty
            if obj.NumNodes==0
                node = [];
            else
                distanceList = obj.distanceFromCoordinates(coordinate(:), obj.NodeList);
                [~,node] = min(distanceList);
            end
            
        end
        
        function dist = distanceBetweenNodes(obj, nodes1, nodes2)
            %distanceBetweenNodes Distance between two given nodes
            %   D = distanceBetweenNodes(G, N1, N2) is the distance between
            %   two given sets of nodes.
            
            pose1 = obj.Nodes(:,nodes1);
            pose2 = obj.Nodes(:,nodes2);
            dist = obj.distanceFromCoordinates(pose1, pose2);
        end
        
        function [distanceList,label] = distanceFromAllNodes(obj, coordinate)
            %distanceFromAllNodes Distances from a given coordinate to all nodes
            %   [D,L] = distances(G, C) is a vector (1xN) of geometric
            %   distance from the point C (Dx1) to every other node sorted
            %   into increasing order with the corresponding node label L.
            %
            %   See also closestNode.
            
            distanceList = obj.distanceFromCoordinates(coordinate(:), obj.NodeList);
            [distanceList,label] = sort(distanceList, 'ascend');
        end
        
        function path = aStar(obj, startNode, goalNode)
            %aStar Perform A* search algorithm
            %   PATH = aStar(G, V1, V2) is the lowest cost path from node V1 to
            %   node V2.  PATH is a list of nodes starting with V1 and ending
            %   V2.
            %
            %   [PATH,C] = aStar(G, V1, V2) as above but also returns the
            %   total cost of traversing PATH.
            
            componentStart = obj.componentFromNodes(startNode);
            componentGoal = obj.componentFromNodes(goalNode);
            if componentStart~=componentGoal
                path = [];
                return;
            end
            
            path = robotics.algs.internal.aStar(startNode, goalNode, ...
                obj.NodeList, obj.EdgeList, logical(obj.IsEuclidean));
        end
    end
    
    %%
    methods (Access='protected')
        function label = newLabel(obj)
            %newLabel Create a new label of the graph
            %   L = newLabel(G) create and return a new label of the graph
            %   CurrentLabel is updated by increment of 1
            
            obj.CurrentLabel = obj.CurrentLabel + 1;
            label = obj.CurrentLabel;
            
            obj.NodeLabelSet(:,end+1) = label;
            
            obj.NumComponents = length(obj.NodeLabelSet);
        end
        
        function merge(obj, label1, label2)
            %merge Merge 2 labels together
            %   merge(G,L1,L2) merge L1 and L2, lowest label dominates
            %   get the dominant and submissive labels
            
            ldom = min(label1, label2);
            lsub = max(label1, label2);
            
            % change all instances of submissive label to dominant one
            obj.NodeLabels(obj.NodeLabels==lsub) = ldom;
            
            % and remove the submissive label from the set of all labels
            
            obj.NodeLabelSet = unique(obj.NodeLabels);
            obj.NumComponents = length(obj.NodeLabelSet);
        end
        
        function dist = distanceFromCoordinates(obj, pose1,pose2)
            %distance  Return distance between two nodes
            %   D = distanceFromCoordinates(G,P1,P2) return distance
            %   between two given sets of coordinates.
            if(obj.IsEuclidean)
                %   Return a list of Euclidean
                %   distance between a pose P1 and a list of other given pose P2.
                dist = sqrt(sum(bsxfun(@minus, pose1, pose2).^2));
            else
                %   Return a list of distances between
                %   a pose P1 and a list of other given pose P2 in SE2 unit.
                difference = bsxfun(@minus, pose1, pose2);
                difference(3,:) = robotics.internal.wrapToPi(difference(3,:));
                dist = sqrt(sum(difference.^2));
            end
        end
    end
end

