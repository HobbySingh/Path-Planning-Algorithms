function [edges,vertices] = rrt_implement(environment,source,goal,delta)

    clc;

    y = 12;
    [max_y, max_x] = size(environment);

    vertices = source; %initialize vertices with source
    tic;
    edges = double.empty(0, 2);
    pause('on');
    imshow(environment)
    disp('Image called');
    axis on;
    grid on ;
    grid minor;
    pause(2);
    hold on
    for n = 1:10000
    random_point = [(max_x).*rand() (max_y).*rand()];  
    [nearest_point, indx_nearest_point] = closest(random_point,vertices); % nearest point from veritces to random point
    new_point = find_new_point(nearest_point, random_point, delta);

    if new_point(1) > max_x || new_point(2) > max_y
        continue;
    end
    tmpx = new_point(1);
    tmpy = new_point(2);
    if(tmpx < 1 || tmpy < 1)
        continue;
    end

    if environment(ceil(new_point(2)),ceil(new_point(1))) == 1
        if edge_in_freespace(environment,nearest_point,new_point) 
            disp('unique point')
            new_point
            vertices = [vertices;new_point];

            [qNewIndex, ~] = size(vertices);
            edges = [edges; [int32(qNewIndex), int32(indx_nearest_point)]];
            rrtDraw(environment, source, goal, vertices, edges);
            if is_goal(new_point, goal) || goal_on_edge(nearest_point, new_point, goal)
                disp('goal found');
                % Fill path and stop break RRT function
                path = fillSolutionPath(edges, vertices);
                simplified_path = simplified(environment,path,vertices,delta);
                map_update(vertices,simplified_path);
                toc;
                return;        

            end
        end
    end

    end
    rrtDraw(environment, source, goal, vertices, edges);
    drawnow

end

% nearest point from already existing vertices to a random point  
function [nearest_point, indx_nearest_point] = closest (random_point, vertices)

    [num_vertices,~] = size(vertices); 
    euclidean_distance = zeros(num_vertices,1);
    for n = 1:num_vertices
        euclidean_distance(n,1) = pdist2(random_point, vertices(n, :), 'euclidean');
    end
    least_distance = min(euclidean_distance);    
    indx_nearest_point = find(euclidean_distance == least_distance,1);
    nearest_point = vertices(indx_nearest_point,:);

end

function new_point = find_new_point (nearest_point,random_point, delta)

    d1 = random_point(1) - nearest_point(1);
    d2 = random_point(2) - nearest_point(2);
    v = [d1,d2];
    u = v / norm(v);
    new_point = (nearest_point + delta * u);

end

function [isBelongsFreeSpace] = edge_in_freespace(environment, nearest_point, new_point)
    
    intermediatePointCount = 10;
    v = new_point - nearest_point;
    distance = norm(v);
    u = v / distance;
    delta_q = distance / intermediatePointCount;
    currentCoordinate = nearest_point;
    for ii = 1 : intermediatePointCount
        currentCoordinate = currentCoordinate + (delta_q * u);
        if environment(ceil(currentCoordinate(2)), ceil(currentCoordinate(1))) == 0 % map(q_new(1), q_new(2))
            disp('Obstacle Hai Yar');
            isBelongsFreeSpace = 0;
            return;
        end
    end
    isBelongsFreeSpace = 1;

end

function [isgoal] = is_goal(new_point, goal)
    
    x = 5;
    deltax = 10/x;deltay = 10/x;
    if ((new_point(1) - deltax)  < goal(1)) && (goal(1) < (new_point(1) + deltax))
        if((new_point(2) - deltay) < goal(2)) && (goal(2) < (new_point(2) + deltay))
            isgoal = 1;
            return;
        end
    end
    isgoal = 0;
    
end
function [isgoalonedge] = goal_on_edge(nearest_point, new_point, goal)

    intermediatePointCount = 10;    
    v = new_point - nearest_point;
    distance = norm(v);
    u = v / distance;
    delta_q = distance / intermediatePointCount;
    currentCoordinate = nearest_point;
    for ii = 1 : intermediatePointCount
        currentCoordinate = currentCoordinate + (delta_q * u);
        if is_goal(new_point,goal) == 1 % map(q_new(1), q_new(2))
            isgoalonedge = 1;
            return;
        end
    end
    isgoalonedge = 0;

end

% Fill path and stop break RRT function
function [path] = fillSolutionPath(edges, vertices)

    disp('Filling');
    path = edges(end, 1);
    prev = edges(end, 2);
    ii = 0;
    [edgesCount, ~] = size(edges);
    
    while prev ~= 1
        if ii > edgesCount
            error('RRT: no path found :(');
        end
        prevIndex = find(edges(:, 1) == prev);
        prev = edges(prevIndex(1), 2);
        path = [path, prev];
        ii = ii + 1;
    end

end

% Plots the RRT result
function rrtDraw(map, q_start, q_goal, vertices, edges)
    
    title('RRT (Rapidly-Exploring Random Trees)');    
    hold on;
    [edgesRowCount, ~] = size(edges);
    
    for ii = 1 : edgesRowCount
        if(ii == edgesRowCount)
            plot([vertices(edges(ii, 1), 1), vertices(edges(ii, 2), 1)], ...
            [vertices(edges(ii, 1), 2), vertices(edges(ii, 2), 2)], ...
             'r', 'LineWidth', 0.2);
        else
            plot([vertices(edges(ii, 1), 1), vertices(edges(ii, 2), 1)], ...
            [vertices(edges(ii, 1), 2), vertices(edges(ii, 2), 2)], ...
             'b', 'LineWidth', 0.2); 
        end       
    end
    
    plot(q_start(1), q_start(2), 'r*', 'linewidth', 0.1);
    drawnow
    plot(q_goal(1), q_goal(2), 'r*', 'linewidth', 0.1);
    drawnow

end

function map_update(vertices,path)
        
    [~, pathCount] = size(path);    
    for ii = 1 : pathCount - 1
        plot([vertices(path(ii), 1), vertices(path(ii + 1), 1)], ...
        [vertices(path(ii), 2), vertices(path(ii + 1), 2)], ...
         'r', 'LineWidth', 2);
    end
    
end
