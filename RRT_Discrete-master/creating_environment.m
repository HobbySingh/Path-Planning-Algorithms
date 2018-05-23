y = 12;
x = 12;
empty_world = ones(int32(150/y),int32(250/y));

% obstacle 1
for i = int32(30/x):int32(45/x) 
    for j = int32(39/x):int32(64/x)
        empty_world(i,j) = 0;
    end
end

% obstacle 2
for i = int32(42/x):int32(77/x) 
    for j = int32(89/x):int32(120/x)
        empty_world(i,j) = 0;
    end
end

% obstacle 3
for i = int32(3):int32(75/x) 
    for j = int32(139/x):int32(182/x)
        empty_world(i,j) = 0;
    end
end

% obstacle 4
for i = int32(95/x):int32(135/x) 
    for j = int32(164/x):int32(182/x)
        empty_world(i,j) = 0;
    end
end

% obstacle 5
for i = int32(3):int32(137/x)
    for j = int32(200/x):int32(220/x)
        empty_world(i,j) = 0;
    end
end

save environment

imshow(empty_world)
axis on 
grid on
grid minor