function plotblock(block)

    xs =[ block(1), block(4)];
    ys =[ block(2), block(5)];
    zs =[ block(3), block(6)];
    block_col = double(block(7:9))/255;
    % generate points
    pts = [[xs(1), ys(1), zs(1)];
         [xs(2), ys(1), zs(1)];
         [xs(2), ys(2), zs(1)];
         [xs(1), ys(2), zs(1)];
         [xs(1), ys(1), zs(2)];
         [xs(2), ys(1), zs(2)];
         [xs(2), ys(2), zs(2)];
         [xs(1), ys(2), zs(2)];
        ];
    %lower
    pts_int = pts(1:4,:);
    drawPlane(pts_int, block_col);
    
    %top
    pts_int = pts(5:8,:);
    drawPlane(pts_int, block_col);
    
    %left
    pts_int = pts([1,4,8,5],:);
    drawPlane(pts_int, block_col);
    
    %right
    pts_int = pts([2,3,7,6],:);
    drawPlane(pts_int, block_col);
    
    %front
    pts_int = pts([1,2,6,5],:);
    drawPlane(pts_int, block_col);
    
    %back
    pts_int = pts([3,7,8,4],:);
    drawPlane(pts_int, block_col);

end

function drawPlane(pts, block_col)
    % draw a plane based on input
    x=zeros(1,4);
    y=zeros(1,4);
    z=zeros(1,4);
    for i = 1:4
        x(i) =  pts(i,1);
        y(i) =  pts(i,2);
        z(i) =  pts(i,3);
    end
    
    fill3(x,y,z,block_col);
end