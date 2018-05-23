
clc
clear
close all

max_x=20;
max_y=20;

environment=2*(ones(max_x,max_y));

j=0;
x_val = 1;
y_val = 1;
axis([1 max_x+1 1 max_y+1])
grid on;
hold on;
n=0;

xval=18;
yval=18;
x_target=xval;
y_target=yval;

environment(xval,yval)=0;%Initialize environment with location of the target
plot(xval+.5,yval+.5,'gd');
text(xval+1,yval+.5,'Target')

% obstacle 1
for i = 3:5
    for j = 13:16
        environment(i,j) = -1; 
        %plot(i+.5,j+.5,'rd'); 
    end
end
x = [3, 6, 6, 3];
y = [13, 13, 17, 17];

fill(x,y,'k');
% obstacle 2
for i = 7:9
    for j = 11:14 
        environment(i,j) = -1;
        %plot(i+.5,j+.5,'rd'); 
    end
end

x = [7, 10, 10, 7];
y = [11, 11, 15, 15];

fill(x,y,'k');

% obstacle 3
for i = 11:13
    for j = 11:17 
        environment(i,j) = -1;
        %plot(i+.5,j+.5,'rd'); 
    end
end
x = [11, 14, 14, 11];
y = [11, 11, 18, 18];

fill(x,y,'k');
% obstacle 4
for i = 12:13
    for j = 6:9 
        environment(i,j) = -1;
        %plot(i+.5,j+.5,'rd'); 
    end
end

x = [12, 14, 14, 12];
y = [6, 6, 10, 10];

fill(x,y,'k');

% obstacle 5
for i = 15:16
    for j = 6:17 
        environment(i,j) = -1;
        %plot(i+.5,j+.5,'rd'); 
    end
end

x = [15, 17, 17, 15];
y = [6, 6, 18, 18];

fill(x,y,'k');

%plot(19+.5,19+.5,'gd');

xval=3;
yval=3;
x_start=xval;%Starting Position
y_start=yval;%Starting Position

environment(xval,yval)=1;
 plot(xval+.5,yval+.5,'bo');

valid=[]; % x | y | parent_x | parent_y | h(n) | g(n) | f(n) 

in_valid=[]; % x | y

k=1;
for i=1:max_x
    for j=1:max_y
        if(environment(i,j) == -1) % check if obstacle
            in_valid(k,1)=i; 
            in_valid(k,2)=j; 
            k=k+1;
        end
    end
end
in_valid_size=size(in_valid,1);

cell_x=x_start;
cell_y=y_start;
valid_size=1; % initialize size of valid_list
path_cost=0;
goal_distance=sqrt((cell_x-x_target)^2 + (cell_y-y_target)^2);

new_row=[1,8];
new_row(1,1)=1;
new_row(1,2)=cell_x;
new_row(1,3)=cell_y;
new_row(1,4)=cell_x; % parent x
new_row(1,5)=cell_y; % parent y 
new_row(1,6)=path_cost;
new_row(1,7)=goal_distance;
new_row(1,8)=goal_distance;

valid(valid_size,:)=new_row; % initializing path with start position
valid(valid_size,1)=0;

in_valid_size=in_valid_size+1;
in_valid(in_valid_size,1)=cell_x; % make it invalid for further iterations
in_valid(in_valid_size,2)=cell_y;

path_not_found=1;

while((cell_x ~= x_target || cell_y ~= y_target) && path_not_found == 1)
    % x | y | h | g | f
    successors=explore_successors(cell_x,cell_y,path_cost,x_target,y_target,in_valid,max_x,max_y);
    successors_size=size(successors,1);
    for i=1:successors_size
    flag=0;
    for j=1:valid_size
        if(successors(i,1) == valid(j,2) && successors(i,2) == valid(j,3) ) % if successor same as already existing cell inpath
%             disp('valid')
%             valid
%             disp(' ');
            valid(j,8)=min(valid(j,8),successors(i,5)); % check for minimum f and then pick it 
            if valid(j,8) == successors(i,5)
                valid(j,4)=cell_x;% parent x
                valid(j,5)=cell_y;% parent y
                valid(j,6)=successors(i,3); % h
                valid(j,7)=successors(i,4); % g
            end;
            flag=1;
        end;
    end;
    if flag == 0 % if new cell with minimum f(n) then add to valid_path
        valid_size= valid_size+1;
        new_row=[1,8];
        new_row(1,1)=1;
        new_row(1,2)=successors(i,1);
        new_row(1,3)=successors(i,2);
        new_row(1,4)=cell_x; % parent x
        new_row(1,5)=cell_y; % parent y
        new_row(1,6)=successors(i,3); % h
        new_row(1,7)=successors(i,4); % g
        new_row(1,8)=successors(i,5); % f
        valid(valid_size,:)= new_row;
     end;
    end;

    index_min_cell = min_f(valid,valid_size,x_target,y_target);
    if (index_min_cell ~= -1) % if index with minimum fn is obstacle no path exists    
        cell_x=valid(index_min_cell,2);
        cell_y=valid(index_min_cell,3);
        path_cost=valid(index_min_cell,6);

        in_valid_size=in_valid_size+1; % put the cell in_valid so we dont come back on it again
        in_valid(in_valid_size,1)=cell_x;
        in_valid(in_valid_size,2)=cell_y;
        valid(index_min_cell,1)=0;
    else
        path_not_found=0;
    end;
end;

% backtracking to find the path

i=size(in_valid,1);
path=[];
xval=in_valid(i,1); % pick last in in_valid_list that must be target
yval=in_valid(i,2);
i=1;
path(i,1)=xval;
path(i,2)=yval;
i=i+1;

if ( (xval == x_target) && (yval == y_target))
    inode=0;
   parent_x=valid(find((valid(:,2) == xval) & (valid(:,3) == yval),1),4);
   parent_y=valid(find((valid(:,2) == xval) & (valid(:,3) == yval),1),5);
   
   while( parent_x ~= x_start || parent_y ~= y_start)
           path(i,1) = parent_x;
           path(i,2) = parent_y;
           
           inode=find((valid(:,2) == parent_x) & (valid(:,3) == parent_y),1);
           parent_x=valid(inode,4);
           parent_y=valid(inode,5);
           i=i+1;
    end;

% plottin path

j=size(path,1);
 p=plot(path(j,1)+.5,path(j,2)+.5,'bo');
  j=j-1;
 for i=j:-1:1
  pause(.25);
  set(p,'XData',path(i,1)+.5,'YData',path(i,2)+.5);
 drawnow ;
 end;
 plot(path(:,1)+.5,path(:,2)+.5);
else
disp( 'Sorry, No path exists to the Target!');
end

