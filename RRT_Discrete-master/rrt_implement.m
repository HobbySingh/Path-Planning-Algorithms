function [edges,vertices] = rrt_implement(environment,source,goal,delta)

% edges - 
% vertices - 
% environment - 
% soturce -
% goal -
% delta - 
clc;

[max_y, max_x] = size(environment);

vertices = source; %initialize vertices with source
%tic;
%imshow(environment);
edges = int32.empty(0, 2);
pause('on');
imshow(environment)
disp('Image called');
pause(2);
hold on
for n = 1:10000
    
    random_point = [randi(max_x) randi(max_y)];
    
    [nearest_point, indx_nearest_point] = closest(random_point,vertices); % nearest point from veritces to random point
    new_point = find_new_point(nearest_point, random_point, delta);
    
    if int32(new_point(1)) > max_x || int32(new_point(2)) > max_y
        continue;
    end
    
%     disp('valueof new point x and y');
    tmpx = int32(new_point(1));
    tmpy = int32(new_point(2));
    if(tmpx < 1 || tmpy < 1)
        continue;
    end
    
    %disp('   ');
    
    if environment(int32(new_point(2)),int32(new_point(1))) == 1
        if edge_in_freespace(environment,nearest_point,new_point) 
                %X = ['Itni bar',num2str(n),'yo'];
                %disp(X);        
%             if find(vertices(:,1) == new_point(1)) > 1 
%                 if find(vertices(:,2) == new_point(2)) > 1
%                     disp('same point')
%                     continue;
%                 end
%             end
            disp('unique point')
            new_point
            vertices = [vertices;new_point];
            
            [qNewIndex, ~] = size(vertices);
            edges = [edges; [int32(qNewIndex), int32(indx_nearest_point)]];
            rrtDraw(environment, source, goal, vertices, edges);
            if isequal(new_point, goal) || goal_on_edge(nearest_point, new_point, goal)

                if ~isequal(new_point, goal) % if goal is not q_new but its on edge
                    vertices = vertices(1 : (end - 1), :);
                    vertices = [vertices; goal];
                end
                
                % Fill path and stop break RRT function
                path = fillSolutionPath(edges, vertices);
                if n == 1
                    rrtDraw(environment, source, goal, vertices, edges);
                    Disp('RRT Draw CHLE HAI');
                end
                %map_update(environment,source,goal,vertices,edges);
                %drawnow
                %toc;
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
        euclidean_distance(n,1) = pdist2(double(random_point), double(vertices(n, :)), 'euclidean');
    end
    
    least_distance = min(euclidean_distance);
    
    indx_nearest_point = find(euclidean_distance == least_distance,1);
    nearest_point = vertices(indx_nearest_point,:);
%     disp('Nearest Point');
%     nearest_point
%     indx_nearest_point
%     disp(' ');
    

end

function new_point = find_new_point (nearest_point,random_point, delta)
    srp = size(random_point);
    snp = size(nearest_point);
%     X = ['Random Point : x - ' , num2str(random_point(1)),' y: - ', num2str(random_point(2)), 'Size RP : ', num2str(srp(1)), 'X', num2str(srp(2))];
%     disp(X);
%     X = ['Nearest Point : x - ' , num2str(nearest_point(1)),' y: - ', num2str(nearest_point(2)),  'Size NP : ', num2str(snp(1)), 'X', num2str(snp(2))];  
%     disp(X);
%     disp('  ');
    d1 = double(random_point(1)) - double(nearest_point(1));
    d2 = double(random_point(2)) - double(nearest_point(2));
    v = [d1,d2];
    
    u = v / norm(v);
    
    new_point = (int32(double(nearest_point) + delta * u));

end

function [isBelongsFreeSpace] = edge_in_freespace(environment, nearest_point, new_point)
    
    intermediatePointCount = 10;
    
    v = double(double(new_point) - double(nearest_point));
    
    distance = norm(v);
    
    u = v / distance;
    
    delta_q = distance / intermediatePointCount;
    
    currentCoordinate = double(nearest_point);
    
    for ii = 1 : intermediatePointCount
        
        currentCoordinate = currentCoordinate + (delta_q * u);
        
        if environment(int32(currentCoordinate(2)), int32(currentCoordinate(1))) == 0 % map(q_new(1), q_new(2))
            disp('Obstacle Hai Yar');
            isBelongsFreeSpace = 0;
            return;
        end
        
    end
    isBelongsFreeSpace = 1;

end

function [isQGoalOnEdge] = goal_on_edge(nearest_point, new_point, goal)
    
    v = double(double(new_point) - double(nearest_point));
    
    distance = norm(v);
    
    u = v / distance;
    
    distanceQNearQGoal = norm(double(double(goal) - double(nearest_point)));
    
    if distanceQNearQGoal > distance
        isQGoalOnEdge = 0;
        return;
    end
    
    goal_2 = double(nearest_point) + (distanceQNearQGoal * u);
    
    isQGoalOnEdge = isequal(int32(goal_2), goal);        

end

% Fill path and stop break RRT function
function [path] = fillSolutionPath(edges, vertices)

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

    %imshow(map);
    title('RRT (Rapidly-Exploring Random Trees)');
    % imagesc(1 - map);
    % colormap(gray);
    
    hold on;
    
    [edgesRowCount, ~] = size(edges);
    
    for ii = 1 : edgesRowCount
        %plot(vertices(ii, 1), vertices(ii, 2), 'cyan*', 'linewidth', 0.1);
        %drawnow
        plot([vertices(edges(ii, 1), 1), vertices(edges(ii, 2), 1)], ...
        [vertices(edges(ii, 1), 2), vertices(edges(ii, 2), 2)], ...
         'b', 'LineWidth', 0.2);
        %drawnow
    end
    
    plot(q_start(1), q_start(2), 'r*', 'linewidth', 0.1);
    drawnow
    plot(q_goal(1), q_goal(2), 'r*', 'linewidth', 0.1);
    drawnow
    
    
%     [~, pathCount] = size(path);
%     
%     for ii = 1 : pathCount - 1
%         %plot(vertices(ii, 1), vertices(ii, 2), 'cyan*', 'linewidth', 1);
%         plot([vertices(path(ii), 1), vertices(path(ii + 1), 1)], ...
%         [vertices(path(ii), 2), vertices(path(ii + 1), 2)], ...
%          'r', 'LineWidth', 2);
%     end
    
end

function map_update(map, q_start, q_goal, vertices, edges)
        
        [edgesRowCount, ~] = size(edges);
        plot(vertices(edgesRowCount, 1), vertices(edgesRowCount, 2), 'cyan*', 'linewidth', 0.1);
        plot([vertices(edges(edgesRowCount, 1), 1), vertices(edges(edgesRowCount, 2), 1)], ...
        [vertices(edges(edgesRowCount, 1), 2), vertices(edges(edgesRowCount, 2), 2)], ...
         'b', 'LineWidth', 0.2);
        drawnow limitrate
end
