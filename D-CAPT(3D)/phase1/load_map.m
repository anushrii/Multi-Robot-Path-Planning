function map = load_map(filename, xy_res, z_res, margin)
%% LOAD_MAP Load a map from disk.
%  MAP = LOAD_MAP(filename, xy_res, z_res, margin).  Creates an occupancy grid
%  map where a node is considered fill if it lies within 'margin' distance of
%  on abstacle.
global margin_extra
margin_extra = 0.2;
[boundary, blocks] = read_map_file(filename);
margin = margin+margin_extra;
margin_extra = margin_extra/3;
map.xy_res = xy_res;
map.z_res = z_res;
map.margin = margin;
map.boundary = boundary;
map.blocks = blocks;
[map.adj_mat, map.linind, map.obstacle_inds, map.dim] = adjMatrix(map);
end

function [adj_mat, linind, obstacle_inds, dim] = adjMatrix(map)
%% Generate an adjacency matrix (sparse) for every node with every node
% in a 26-connected grid with the distances as the edge weights.

boundary = map.boundary;
xy_res = map.xy_res;
z_res = map.z_res;

n_connected = 26;

nx = floor( (boundary(4)-boundary(1) )/ xy_res ) +1 ; % xmax-xmin/xres
ny = floor( (boundary(5)-boundary(2) )/ xy_res ) +1; % ymax-ymin/yres
nz = floor( (boundary(6)-boundary(3) )/ z_res ) +1; % zmax-zmin/zres
dim = [nx,ny,nz];
% create all nodes meshgrid
num_nodes = nx * ny * nz ;
[x_mesh,y_mesh,z_mesh] = meshgrid(1:nx,1:ny,1:nz);
x_lin = x_mesh(:);
y_lin = y_mesh(:);
z_lin = z_mesh(:);
all_pts = [x_lin, y_lin, z_lin];

x_adj = zeros(num_nodes, n_connected);
y_adj = zeros(num_nodes, n_connected);
z_adj = zeros(num_nodes, n_connected);

dist = zeros(n_connected,1);
l = 1;

% 26 connected meshgrid inds
% dist - dist of adj nodes
for i = -1:1
    for j = -1:1
        for k = -1:1
            if(~(i==0 && j==0 && k==0))
                x_adj(:,l) = validIndex( x_lin+i, nx);
                y_adj(:,l) = validIndex( y_lin+j, ny);
                z_adj(:,l) = validIndex( z_lin+k, nz);
                dist(l) = sqrt( (i*xy_res)^2 + (j*xy_res)^2 + (k*z_res)^2 );
                l = l+1;
            end
        end
    end
end

adj_mat_row = zeros(num_nodes* n_connected,1);
adj_mat_col = zeros(num_nodes* n_connected,1);
adj_mat_val = zeros(num_nodes* n_connected,1);

linind = sub2ind([nx,ny,nz], x_lin, y_lin, z_lin);
for i = 1:n_connected
    st_ind = (i-1)*num_nodes+1;
    end_ind = i*num_nodes;
    adj_mat_row(st_ind:end_ind) = linind;
    adj_mat_col(st_ind:end_ind) = sub2ind([nx,ny,nz], x_adj(:,i), y_adj(:,i), z_adj(:,i)); 
    adj_mat_val(st_ind:end_ind) = ones(num_nodes,1)*dist(i);
end


% remove out of bounds inds 
bad_inds = isnan(adj_mat_col);

adj_mat_row(bad_inds) = [];
adj_mat_col(bad_inds) = [];
adj_mat_val(bad_inds) = [];

adj_mat = sparse(adj_mat_row, adj_mat_col,adj_mat_val, ...
                num_nodes, num_nodes);

% remove obstacle edges
 [obstacle_inds] = obstaclePoints(map, all_pts);
if nnz(obstacle_inds)
 obstacle_lin  = linind(obstacle_inds);
 adj_mat(obstacle_lin,:) = adj_mat(obstacle_lin,:)*0;
 adj_mat(:,obstacle_lin) = adj_mat(:,obstacle_lin)*0; 
end

end

function [dangerous_pts] = obstaclePoints(map, all_pts)
%% get node indices which are in obstacle_inds
    all_pts_coords = subs2xyz(map, all_pts);
    
    margin = map.margin;
    blocks = map.blocks;
    boundary = map.boundary;
    n = size(all_pts_coords,1);
    obstacle_inds = false(n,1);
    
    for i =1:size(blocks,1)
        obstacle_inds = obstacle_inds | ...
            ( ( all_pts_coords(:,1) >= (blocks(i,1) -margin) ) & ( all_pts_coords(:,1) <= (blocks(i,4) +margin) ) & ...
              ( all_pts_coords(:,2) >= (blocks(i,2) -margin) ) & ( all_pts_coords(:,2) <= (blocks(i,5) +margin) ) & ...
              ( all_pts_coords(:,3) >= (blocks(i,3) -margin) ) & ( all_pts_coords(:,3) <= (blocks(i,6) +margin) ) ...  
            );
%         obstacle_inds = obstacle_inds |  ( all_pts_coords(:,3) <=  (1*margin))  | ... 
%                                     ( all_pts_coords(:,3) >= (boundary(6) -(1*margin))  ) ;
    end
    %[dangerous_pts] = pointsMarginAwayFromObstacles(pts_inside_obstacle_coords , all_pts_coords, margin);
    dangerous_pts = obstacle_inds;
end

function x = validIndex(x,xmax)
    %% Makes indices out of legal range nan
    x(x<1 | x>xmax) = NaN;
end

