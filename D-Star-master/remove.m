function [valid,valid_size] = remove(valid,cell_x,cell_y)
    [~ ,num_col] = size(valid);
    if num_col>3
        fprintf('removing from valid x %d, y %d\n',cell_x,cell_y);
    end
    %find((valid(:,2) == cell_x) & (valid(:,3) == cell_y),1)
    valid(find((valid(:,2) == cell_x) & (valid(:,3) == cell_y),1),:) = [];
   [valid_size,~] = size(valid);
   disp('Valid after removing')
   valid
end