y = 5;
x = 12;

world_offset_x = 150/y;
world_offset_y = 250/y;

empty_world = ones(int32(world_offset_x),int32(world_offset_y));

% obstacle 1
for i = int32(world_offset_x*0.25):int32(world_offset_x*0.32) 
    for j = int32(world_offset_y*0.2):int32(world_offset_y*0.27)
        empty_world(i,j) = 0;
    end
end

% obstacle 2
for i = int32(world_offset_x*0.3):int32(world_offset_x*0.4) 
    for j = int32(world_offset_y*0.35):int32(world_offset_y*0.45)
        empty_world(i,j) = 0;
    end
end

% obstacle 3
for i = int32(world_offset_x*0.25):int32(world_offset_x*0.45) 
    for j = int32(world_offset_y*0.55):int32(world_offset_y*0.65)
        empty_world(i,j) = 0;
    end
end

% obstacle 4
for i = int32(world_offset_x*0.57):int32(world_offset_x*0.78) 
    for j = int32(world_offset_y*0.58):int32(world_offset_y*0.65)
        empty_world(i,j) = 0;
    end
end

% obstacle 5
for i = int32(world_offset_x*0.25):int32(world_offset_x*0.78)
    for j = int32(world_offset_y*0.75):int32(world_offset_y*0.82)
        empty_world(i,j) = 0;
    end
end

save environment

imshow(empty_world)
axis on 
grid on
grid minor