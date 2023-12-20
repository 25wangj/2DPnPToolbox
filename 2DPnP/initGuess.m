function pose = initGuess(worldPts, rotImgPts, beta)
    %Find the three candidate points
   [~,i1] = min(rotImgPts(1,:));
   [~,i2] = max(rotImgPts(1,:));
   [~,i3] = max(abs((worldPts(1,:) - worldPts(1,i1)) * (worldPts(2,i2) - worldPts(2,i1))...
                   -(worldPts(2,:) - worldPts(2,i1)) * (worldPts(1,i2) - worldPts(1,i1))));
   p = worldPts(:,[i1 i2 i3]);
   q = rotImgPts(:,[i1 i2 i3]);
   %Compute projections of q onto the xy plane
   r = [cos(beta) 0 sin(beta);0 1 0;-sin(beta) 0 cos(beta)] * q;
   cs = normc(r(1:2,:));
   s12 = cs(1,1) * cs(2,2) - cs(2,1) * cs(1,2);
   s23 = cs(1,2) * cs(2,3) - cs(2,2) * cs(1,3);
   s31 = cs(1,3) * cs(2,1) - cs(2,3) * cs(1,1);
   %Compute heading angle
   theta = atan(((p(2,1) * cs(1,1) - p(1,1) * cs(2,1)) * s23...
        + (p(2,2) * cs(1,2) - p(1,2) * cs(2,2)) * s31...
        + (p(2,3) * cs(1,3) - p(1,3) * cs(2,3)) * s12)...
        / ((p(1,1) * cs(1,1) + p(2,1) * cs(2,1)) * s23... 
        + (p(1,2) * cs(1,2) + p(2,2) * cs(2,2)) * s31...
        + (p(1,3) * cs(1,3) + p(2,3) * cs(2,3)) * s12));
   %Compute x and y
   st = sin(theta);
   ct = cos(theta);
   st1 = st * cs(1,1) + ct * cs(2,1);
   ct1 = ct * cs(1,1) - st * cs(2,1);
   st2 = st * cs(1,2) + ct * cs(2,2);
   ct2 = ct * cs(1,2) - st * cs(2,2);
   x = (-p(1,1) * st1 * ct2 + p(1,2) * ct1 * st2 + p(2,1) * ct1 * ct2 - p(2,2) * ct1 * ct2) / s12;
   y = (-p(1,1) * st1 * st2 + p(1,2) * st1 * st2 + p(2,1) * ct1 * st2 - p(2,2) * st1 * ct2) / s12;
   %Choose the best solution based on reprojection error
   if norm(rpError(p, q, [x y theta], beta)) < norm(rpError(p, q, [x y theta+pi], beta))
        pose = [x y theta];
   else
        pose = [x y theta+pi];
   end
end