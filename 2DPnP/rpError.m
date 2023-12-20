function [err, jac] = rpError(worldPts, rotImgPts, pose, beta)
    len = size(worldPts, 2);
    err = zeros(2 * len, 1);
    if nargout > 1
        jac = zeros(2 * len, 3);
    end
    ct = cos(pose(3));
    st = sin(pose(3));  
    sb = sin(beta);
    cb = cos(beta);
    for i = 1:len
        %Compute reprojection error components
        xi = pose(1) - worldPts(1,i);
        yi = pose(2) - worldPts(2,i);
        zi = worldPts(3,i);
        s1 = xi * ct + yi * st;
        s2 = yi * ct - xi * st;
        denom = s1 * sb - zi * cb;
        err(2 * i - 1) = rotImgPts(1,i) - (s1 * cb + zi * sb) / denom;
        err(2 * i) = rotImgPts(2,i) - s2 / denom;
        if nargout > 1
            %Compute jacobian components
            jac(2 * i - 1, 1) = zi * ct / denom^2;
            jac(2 * i - 1, 2) = zi * st / denom^2;
            jac(2 * i - 1, 3) = zi * s2 / denom^2;
            jac(2 * i, 1) = (yi * sb - zi * cb * ct) / denom^2;
            jac(2 * i, 2) = (zi * cb * ct - xi * sb) / denom^2;
            jac(2 * i, 3) = (sb * (xi^2 + yi^2)) / denom^2;
        end
    end
end