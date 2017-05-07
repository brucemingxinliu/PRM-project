PRM.m

classdef (Sealed) PRM < handle & robotics.algs.internal.GridAccess & matlab.mixin.Copyable
    
    
    properties (Dependent)

        Map
        

        NumNodes
    end
    
    properties

        ConnectionDistance = inf;
    end
    
    properties (Access = private)
        
        %InternalNumNodes Internal property that holds the NumNodes
        InternalNumNodes
        
        %UpdateFlag Internal flag to indicate a property has been modified
        UpdateFlag
        
        %LastFoundPath Cache to keep the last found path
        LastFoundPath = []
    end
    
    properties (Access = ?matlab.unittest.TestCase)
        %InternalMap Internal property that holds the map object
        InternalMap
        
        %PathFinder Handle to PathFinder System object
        PathFinder
        
        %RoadmapBuilder Handle to RoadmapBuilder System object
        RoadmapBuilder
    end
    
    methods
        function obj = PRM(map, numnodes)
            %PRM Constructor
            
            % Use empty as default values
            defaults.Map = robotics.BinaryOccupancyGrid.empty(0,1);
            defaults.NumNodes = 50;
            
            % Parse optional inputs
            % Using nargin to throw better errors
            if nargin == 0
                map = defaults.Map;
                numnodes = defaults.NumNodes;
            elseif nargin == 1
                [map, numnodes] = obj.parsePRMInputs(defaults, map);
            else
                [map, numnodes] = obj.parsePRMInputs(defaults, map, numnodes);
            end
            
            % Assign inputs to the properties
            obj.InternalMap = map.copy;
            obj.InternalNumNodes = numnodes;
            obj.UpdateFlag = true;
            
            % Create internal objects for update and findpath
            obj.RoadmapBuilder = robotics.algs.internal.RoadmapBuilder;
            obj.PathFinder = robotics.algs.internal.PathFinder;
        end
        
        function set.Map(obj, map)
            %set.Map Setter for dependent property Map
            validateattributes(map, {'robotics.BinaryOccupancyGrid'}, ...
                {'nonempty', 'scalar'});
            obj.InternalMap = map.copy;
            obj.UpdateFlag = true;
        end
        
        function map = get.Map(obj)
            %get.Map Getter for dependent property Map
            map = obj.InternalMap.copy;
        end
        
        function set.NumNodes(obj, nodes)
            %set.NumNodes Setter for dependent property NumNodes
            validateattributes(nodes, {'double'}, {'nonempty', 'positive', ...
                'scalar', 'integer'});
            obj.InternalNumNodes = nodes;
            obj.UpdateFlag = true;
        end
        
        function nodes = get.NumNodes(obj)
            %get.NumNodes Getter for dependent property NumNodes
            nodes = obj.InternalNumNodes;
        end
        
        function set.ConnectionDistance(obj, dist)
            %set.ConnectionDistance Setter for ConnectionDistance
            
            validateattributes(dist, {'double'}, {'nonnan', 'nonempty', 'positive', ...
                'scalar', 'real'});
            obj.ConnectionDistance = dist;
            obj.setUpdateFlag(true);
        end
        
        function update(obj)
            
            if isempty(obj.InternalMap)
                error(message('robotics:robotalgs:prm:EmptyMap'));
            end
            
            % Construct the roadmap
            obj.RoadmapBuilder.step(obj.InternalMap, ...
                obj.InternalNumNodes, obj.ConnectionDistance);
            
            % Set actual number of nodes
            obj.InternalNumNodes = obj.RoadmapBuilder.Roadmap.NumNodes;
            
            obj.UpdateFlag = false;
            
            % Clear the last found path if any
            obj.LastFoundPath = [];
        end
        
        function path = findpath(obj, start, goal)
            %findpath Find an obstacle free path between two points
            %   XY = findpath(PRM, START, GOAL) computes an obstacle free
            %   path between a 2-by-1 array, START, and a 2-by-1 array, GOAL.
            %   If START and GOAL are not connected in the roadmap, an empty
            %   array will be returned indicating the failure to find a
            %   path. Updating the roadmap using the update function or
            %   increasing the NumNodes may help find a path in a map
            %   with narrow passages
            %
            %   Example:
            %       % Create a binary occupancy grid
            %       map = robotics.BinaryOccupancyGrid(10, 10);
            %
            %       % Create a probabilistic roadmap
            %       prm = robotics.PRM;
            %
            %       % Assign the map to PRM
            %       prm.Map = map;
            %
            %       % Find a path between two points
            %       xy = findpath(prm, [0 0], [10 10]);
            %
            %       % Visualize the path using show
            %       show(prm);
            %
            %   See also robotics.PRM, update, show
            
            % Validate start and goal points
            obj.validateStartGoal(start, goal);
            
            % If not updated, update the roadmap
            if obj.UpdateFlag
                obj.update;
            end
            
            % Find the path
            path = obj.PathFinder.step(obj.RoadmapBuilder.Roadmap, ...
                obj.InternalMap, start, goal);
            
            obj.LastFoundPath = path;
        end
        
        function ax = show(obj, varargin)
            %show Show the map, roadmap and path in a figure
            %   show(PRM) shows the map and the roadmap in a figure. update
            %   will be called if no roadmap exists. If a
            %   path was computed before calling the show method, the path
            %   will also be plotted on the figure.
            %
            %   AH = show(PRM) returns the handle of the axes used for
            %   by the show function.
            %
            %   show(___,Name,Value) provides additional options specified
            %   by one or more Name,Value pair arguments. Name must appear
            %   inside single quotes (''). You can specify several name-value
            %   pair arguments in any order as Name1,Value1,...,NameN,ValueN:
            %
            %       'Parent'      - Handle of an axes that specifies
            %                       the parent of the objects created by
            %                       show.
            %
            %       'Map'         - A string to turn on or off the display
            %                       of the map, whose value is 'on' or 'off'.
            %
            %                       Default: 'on'
            %
            %       'Roadmap'     - A string to turn on or off the display
            %                       of the roadmap, whose value is 'on' or 'off'.
            %
            %                       Default: 'on'
            %
            %       'Path'        - A string to turn on or off the display
            %                       of the path, whose value is 'on' or 'off'.
            %
            %                       Default: 'on'
            %
            %   Example:
            %       % Create a binary occupancy grid
            %       map = robotics.BinaryOccupancyGrid(10, 10);
            %
            %       % Create a probabilistic roadmap
            %       prm = robotics.PRM;
            %
            %       % Assign the map to PRM
            %       prm.Map = map;
            %
            %       % Construct the roadmap by calling update
            %       xy = findpath(prm, [0 0], [10 10]);
            %
            %       % Visualize the path using show
            %       show(prm);
            %
            %       % Visualize the path without the roadmap
            %       show(prm, 'Roadmap', 'off');
            %
            %   See also robotics.PRM, findpath, update
            
            % Input defaults
            inputParams.axHandle = [];
            inputParams.showMap = true;
            inputParams.showRoadmap = true;
            inputParams.showPath = true;
            
            % Parse input arguments if any
            if ~isempty(varargin)
                inputParams = obj.parseShowInputs(varargin{:});
            end
            
            % Update the roadmap if not updated
            if obj.UpdateFlag
                obj.update;
            end
            
            axHandle = inputParams.axHandle;
            
            % Create a new axes if not assigned
            if isempty(axHandle)
                axHandle = newplot;
            end
            
            % Get the hold status for given axes
            holdStatus = ishold(axHandle);
            
            % Plot the map
            if inputParams.showMap
                obj.Map.show('world','Parent', axHandle);
                title(axHandle, message('robotics:robotalgs:prm:FigureTitle').getString);
                hold(axHandle, 'on');
            end
            
            plotColors.gray = [0.6 0.6 0.6];
            plotColors.blue = [0.15 0.25 0.8];
            plotColors.green = [0.150 0.85 0.20];
            
            % Plot the roadmap
            if inputParams.showRoadmap
                if  ~isempty(obj.RoadmapBuilder.Roadmap.EdgeList)
                    % The plot arguments
                    args = {'LineStyle', '-', ...
                        'Color', plotColors.gray};
                    
                    % Plot edges
                    v1 = obj.getEdges();
                    plot(axHandle,v1(1, :), v1(2, :), args{:});
                end
                
                % Plot nodes if the edgelist is empty
                args = {'Marker', 'o', ...
                    'MarkerFaceColor', plotColors.blue, ...
                    'MarkerEdgeColor', plotColors.blue};
                markerSize = 10;
                
                nodeList = obj.RoadmapBuilder.Roadmap.NodeList;
                scatter(axHandle,nodeList(1, :),nodeList(2, :), markerSize, args{:});
                hold(axHandle, 'on');
            end
            
            % Plot the path if there is a last found path
            if inputParams.showPath && ~isempty(obj.LastFoundPath)
                args = {'LineStyle', '-', ...
                    'LineWidth', 2, ...
                    'Marker', 'o', ...
                    'MarkerFaceColor', plotColors.green, ...
                    'MarkerSize', 4,...
                    'MarkerEdgeColor', plotColors.green, ...
                    'Color', plotColors.green};
                
                plot(axHandle, obj.LastFoundPath(:,1),obj.LastFoundPath(:,2),args{:});
            end
            
            % Restore the hold status of the original figure
            if ~holdStatus
                hold(axHandle, 'off');
            end
            
            % Only return handle if user requested it.
            if nargout > 0
                ax = axHandle;
            end
        end
    end
    
    methods(Access = protected)
        function cpObj = copyElement(obj)
            %copyElement Override copyElement method
            % Make a shallow copy of all four properties
            cpObj = copyElement@matlab.mixin.Copyable(obj);
            % Make a deep copy of the handles
            cpObj.PathFinder = obj.PathFinder.clone;
            cpObj.RoadmapBuilder = obj.RoadmapBuilder.clone;
            cpObj.InternalMap = obj.InternalMap.copy;
        end
    end
    
    methods(Access = private)
        function setUpdateFlag(obj, val)
            %setUpdateFlag Allows setting the update flag property
            obj.UpdateFlag = val;
        end
        
        function v1 = getEdges(obj)
            %getEdges Get edges from the roadmap
            i = 1;
            v1 = zeros(2, 3*size(obj.RoadmapBuilder.Roadmap.EdgeList,2));
            for e=obj.RoadmapBuilder.Roadmap.EdgeList
                v1(:, i) = obj.RoadmapBuilder.Roadmap.NodeList(:,e(1));
                v1(:, i+1) = obj.RoadmapBuilder.Roadmap.NodeList(:,e(2));
                v1(:, i+2) = [NaN;NaN];
                i = i+3;
            end
        end
        
        function validateStartGoal(obj, start, goal)
            %validateStartGoal Validate start and goal points
            
            validateattributes(start, {'double'}, {'real', 'finite','numel', 2},...
                'findpath', 'start');
            validateattributes(goal, {'double'}, {'real', 'finite','numel', 2},...
                'findpath', 'goal');
            
            % Indirectly check if start and goal are within the map 
            % Not to replicate error message, throw error received from Map
            try                
                obj.InternalMap.world2grid(start);
                obj.InternalMap.world2grid(goal);
            catch ex
                throw(ex);
            end           
            
            % Verify if points are in free space

            % Using raycast to ensure consistent behavior between input
            % validation and path computation methods. The consistent
            % behavior is that if a point is on an obstacle edge or 
            % obstacle corner, it will be considered occupied.
            
            isStartFree = robotics.algs.internal.raycast(start, start, ...
                obj.InternalMap.Grid, obj.InternalMap.Resolution, ...
                obj.InternalMap.GridLocationInWorld);
                
            if ~isStartFree
                error(message('robotics:robotalgs:prm:OccupiedLocation',...
                    'start'));
            end
            
            isGoalFree = robotics.algs.internal.raycast(goal, goal, ...
                obj.InternalMap.Grid, obj.InternalMap.Resolution, ...
                obj.InternalMap.GridLocationInWorld);
            
            if ~isGoalFree
                error(message('robotics:robotalgs:prm:OccupiedLocation',...
                    'goal'));
            end
        end
    end
    
    methods (Static, Access = private)
        function inputParams = parseShowInputs(varargin)
            %parseShowInputs Parse inputs for show method
            
            % Define the input parser
            validStrings = {'on', 'off'};
            p = inputParser;
            addParameter(p, 'Parent', [], ...
                @(x)robotics.internal.validation.validateAxesHandle(x));
            addParameter(p, 'Map', 'on', ...
                @(x)any(validatestring(x,validStrings)));
            addParameter(p, 'Roadmap', 'on', ...
                @(x)any(validatestring(x,validStrings)));
            addParameter(p, 'Path', 'on', ...
                @(x)any(validatestring(x,validStrings)));
            
            % Parse inputs
            p.parse(varargin{:});
            inputParams.axHandle = p.Results.Parent;
            inputParams.showMap = strcmpi(p.Results.Map,'on');
            inputParams.showRoadmap = strcmpi(p.Results.Roadmap,'on');
            inputParams.showPath = strcmpi(p.Results.Path,'on');
        end
        
        function [map, numnodes] = parsePRMInputs(defaults, varargin)
            %parsePRMInputs Parse constructor inputs for PRM
            
            p = inputParser;
            addOptional(p, 'Map', defaults.Map, @(x)validateattributes(x,...
                {'robotics.BinaryOccupancyGrid'}, {'nonempty', 'scalar'}, 'PRM', 'Map', 1));
            addOptional(p,'NumNodes', defaults.NumNodes, ...
                @(x)validateattributes(x, {'double'}, ...
                {'scalar', 'integer', 'positive','finite'}, 'PRM', 'NumNodes', 2));
            
            % Parse inputs
            parse(p, varargin{:});
            map = p.Results.Map;
            numnodes = p.Results.NumNodes;
        end
    end
end
