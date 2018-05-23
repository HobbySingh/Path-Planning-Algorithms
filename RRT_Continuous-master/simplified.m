function [path_smooth] = simplified(map, path, vertices, delta)

path_smooth = path(1); % initing with goal
currentIndex = 1; % path array iterator
currentSmoothIndex = numel(path); % path reverse array iterator

while currentIndex < numel(path)
    
    while currentIndex < currentSmoothIndex
        
        if isEdgeBelongsFreeSpace(map, vertices(path(currentSmoothIndex), :), vertices(path(currentIndex), :))
            path_smooth = [path_smooth, path(currentSmoothIndex)];
            currentIndex = currentSmoothIndex;
            break;
        else
            currentSmoothIndex = currentSmoothIndex - 1;
        end
        
    end
    
    currentSmoothIndex = numel(path);
    
end

% rrtSmoothDraw(map, path_smooth, vertices);

end

function [isBelongsFreeSpace] = isEdgeBelongsFreeSpace(environment, nearest_point, new_point)
    
    intermediatePointCount = 20;
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

function rrtSmoothDraw(map, path_smooth, vertices)

    imshow(int32(1 - map), []);
    title('RRT (Rapidly-Exploring Random Trees) - Smooth Path');
    % imagesc(1 - map);
    % colormap(gray);
    
    hold on;

    [~, pathCount] = size(path_smooth);
    
    for ii = 1 : pathCount - 1
        %plot(vertices(ii, 1), vertices(ii, 2), 'cyan*', 'linewidth', 1);
        plot([vertices(path_smooth(ii), 1), vertices(path_smooth(ii + 1), 1)], ...
        [vertices(path_smooth(ii), 2), vertices(path_smooth(ii + 1), 2)], ...
         'r', 'LineWidth', 2);
    end
    
end