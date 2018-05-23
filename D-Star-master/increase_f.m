function valid = increase_f(valid,cell_x,cell_y,in_valid,max_x,max_y)
%    fprintf('Increasing child of X : %d Y : %d from valid\n',cell_x,cell_y);
    list_children = find((valid(:,4) == cell_x) & (valid(:,5) == cell_y));
    fprintf(' Found or not %d',sum(find((valid(:,2) == cell_x) & (valid(:,3) == cell_y),1)));
    valid(find((valid(:,2) == cell_x) & (valid(:,3) == cell_y),1),8) = 1000;
    valid(find((valid(:,2) == cell_x) & (valid(:,3) == cell_y),1),1) = 1;
    %valid
    fprintf('Increasing  X : %d Y : %d from valid\n',valid(find(valid(:,2) == cell_x,1),2),valid(find(valid(:,3) == cell_y,1),3));    
    for i = 1:size(list_children)
        valid = increase_f(valid,valid(list_children(i),2),valid(list_children(i),3),in_valid,max_x,max_y);
    end
%     flag = 0;
%     %find((valid(:,2) == cell_x) & (valid(:,3) == cell_y),1)
%     if (sum(find((valid(:,2) == cell_x) & (valid(:,3) == cell_y),1)) == 0) 
%         disp('Came but went');
%         return
%     end
%     
%     fprintf('Increasing X : %d Y : %d from invalid\n',cell_x,cell_y);
%     c2=size(in_valid,1);
%     for k= 1:-1:-1
%         for j= 1:-1:-1
%             if (k~=j || k~=0)  %The node itself is not its successor
%                 s_x = cell_x+k;
%                 s_y = cell_y+j;
%                 if (valid(find((valid(:,2) == s_x) & (valid(:,3) == s_y),1),8) == 10000)
%                     return
%                 end
%                 if( (s_x >0 && s_x <=max_x) && (s_y >0 && s_y <=max_y)) % successor within the matrix
%                     flag=1;                    
%                     for c1=1:c2
%                         if(s_x == in_valid(c1,1) && s_y == in_valid(c1,2)) % successor not an obstacle or already visited
%                             flag=0;
%                         end;
%                     end;
%                     if (flag == 1)
%                         valid(find((valid(:,2) == s_x) & (valid(:,3) == s_y),1),8) = 10000;
%                         valid = increase_f(valid,s_x,s_y,in_valid,max_x,max_y);
%                     end
%                 end
%             end
%         end
%     end    
    
end