function Optimal_path = path_from_A_star(map)
    isDebug = 0;
    isASTAR = 1; % 0 for Dijkstra 1 for A*
    is3D = 1; % 0 for 2D 1 for 3D
    Optimal_path = [];
    size_map = size(map,1);

    MAX_X=10;
    MAX_Y=10;
    MAX_Z=10;
    
    %Define the 3D grid map array.
    %Obstacle=-1, Target = 0, Start=1, free = 2
    MAP=2*(ones(MAX_X,MAX_Y,MAX_Z));
    
    %Initialize MAP with location of the target
    xval=floor(map(size_map, 1));
    yval=floor(map(size_map, 2));
    zval=floor(map(size_map, 3));
    
    xTarget=xval;
    yTarget=yval;
    zTarget=zval;
    tar_position = [xTarget,yTarget,zTarget];
    MAP(xval,yval,zval)=0;
    
    %Initialize MAP with location of the obstacle
    for i = 2: size_map-1
        xval=floor(map(i, 1));
        yval=floor(map(i, 2));
        zval=floor(map(i, 3));
        MAP(xval,yval,zval)=-1;
    end 
    
    %Initialize MAP with location of the start point
    xval=floor(map(1, 1));
    yval=floor(map(1, 2));
    zval=floor(map(1, 3));
    xStart=xval;
    yStart=yval;
    zStart=zval;
    start_position = [xStart,yStart,zStart];
    MAP(xval,yval,zval)=1;
%     tar_position = [xStart,yStart,zStart];
    % Main structure in the A* search =====================================================

    % Container storing nodes to be expanded, along with the f score (f=g+h)
    % Each node's (x,y,z) coordinate and its f score is stored in a row
    % For example, queue = [x1, y1, z1, f1; x2, y2, z2, f2; ...; xn, yn, zn, fn]
    queue = [];  

    % Arrays for storing the g score of each node, g score of undiscovered nodes is inf
    g = inf(MAX_X,MAX_Y,MAX_Z);

    % Arrays recording whether a node is expanded (popped from the queue) or not
    % expanded: 1, not expanded: 0
    expanded = zeros(MAX_X,MAX_Y,MAX_Z);

    % Arrays recording the parent of each node
    parents = zeros(MAX_X,MAX_Y,MAX_Z, 3);
    
    
    %Start your code here ==================================================================
    % TODO
    % init quenue with start point
    queue = [xStart,yStart,zStart,0];
    search_count = 0;
    while 1
        search_count = search_count + 1;
        if size(queue,1) == 0               % break if empty
            disp('No path from S to G!')
            break
        end
        if isDebug == 1
            disp('####################################')
            disp('queue:')
            disp(queue)
        end
  
        [index,~] = find(queue(:,4) == min(queue(:,4)));
        curr_position = queue(index(1),1:3);
        if isDebug == 1
            disp('current position:')
            disp(curr_position)
        end
        g_n = queue(index(1),4);
        queue(index(1),:) = [];                 % remove lowest g(n)
        expanded(curr_position(1),curr_position(2),curr_position(3)) = 1;
        if curr_position == tar_position
            while 1
                Optimal_path = [curr_position;Optimal_path];
                if curr_position == start_position
                    break
                end
                x = curr_position(1); y = curr_position(2); z = curr_position(3);
                curr_position = [parents(x,y,z,1),parents(x,y,z,2),parents(x,y,z,3)];

            end
            disp('Find the path')
            break
        end
        % get neighbors of current position
        neighbors = get_neighbors(curr_position,MAP,is3D);
        if isDebug == 1
            disp('neighbors:')
            disp(neighbors)
        end
        
        for i = 1 : size(neighbors,1)     
            x = neighbors(i,1);y = neighbors(i,2);z = neighbors(i,3);
            if expanded(x,y,z) == 0
                if isASTAR == 1
                    g_m = g_n + 1 + h(neighbors(i),tar_position);
                else
                    g_m = g_n + 1 ;
                end
                if g(x,y,z) == inf
                    queue = [[neighbors(i,:),g_m];queue];
                end
                if g(x,y,z) > g_n + 1
                    parents(x,y,z,:) = curr_position;
                    g(x,y,z) = g_m;
                end
            end
        end
    end
    disp(['total search count: ',num2str(search_count)]);
end
% get neighbor points of current postion
function neighbors = get_neighbors(p,MAP, is3D)
    if is3D == 1
        neighbors = [p(1)+1 p(2) p(3) ; ... 
                     p(1)-1 p(2) p(3) ; ... 
                     p(1) p(2)+1 p(3) ; ... 
                     p(1) p(2)-1 p(3) ; ... 
                     p(1) p(2) p(3)+1 ; ... 
                     p(1) p(2) p(3)-1 ];   
    else
       neighbors = [p(1)+1 p(2) p(3) ; ... 
                 p(1)-1 p(2) p(3) ; ... 
                 p(1) p(2)+1 p(3) ; ... 
                 p(1) p(2)-1 p(3) ];   
    end
   index = [];
   for i = 1 : size(neighbors,1)
       if min(neighbors(i,:))<1 || max(neighbors(i,:))> 10 || ...
               MAP(neighbors(i,1),neighbors(i,2),neighbors(i,3)) == -1
           index = [index;i];
       end
   end
   index = sort(index, 'descend');
   for i = 1 : size(index,1)
       neighbors(index(i),:)=[];
   end
end
% heuristic function for A*
function value = h(curr_point, tar_point)
    value = sum(abs(tar_point-curr_point));
end
