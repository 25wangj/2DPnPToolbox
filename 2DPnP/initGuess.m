function pose = initGuess(worldPts, rotImgPts, beta)
   len = size(worldPts, 2);
   %Find the spherical angles of each camera ray
   worldImgPts = [cos(beta) 0 sin(beta);0 1 0;-sin(beta) 0 cos(beta)] * rotImgPts;
   [thetas, phis, ~] = cart2sph(worldImgPts(1,:), worldImgPts(2,:), worldImgPts(3,:));
   c1 = 0; c2 = 0; c3 = 0; c4 = 0; a1 = 0; a2 = 0; b1 = 0; b2 = 0;
   %Find the coefficients of the polynomial system
   for i=1:len
      xi = worldPts(1,i);
      yi = worldPts(2,i);
      zi = worldPts(3,i);
      if (phis(i) / worldPts(3,i) > 0)
        sp = sin(phis(i));
        cp = cos(phis(i));
        i1 = sp^3 / (cp * zi^2);
        i2 = -2 * xi * i1;
        i3 = -2 * yi * i1;
        i4 = (xi^2 + yi^2) * i1 - sp * cp;
        c1 = c1 + 4 * i1^2;
        c2 = c2 + 2 * i2 * i3;
        c3 = c3 + 2 * i1 * i2;
        c4 = c4 + 2 * i1 * i3;
        a1 = a1 + 4 * i1 * i4 + 2 * i2^2;
        a2 = a2 + 2 * i2 * i4;
        b1 = b1 + 4 * i1 * i4 + 2 * i3^2;
        b2 = b2 + 2 * i3 * i4;
      end
   end
   %Find the coefficients of each resultant
   xcoeffs = [a1^2*c1^3-2*a1*b1*c1^3-4*a1*c1^2*c3^2+4*a1*c1^2*c4^2+b1^2*c1^3+4*b1*c1^2*c3^2-4*b1*c1^2*c4^2+4*c1^3*c2^2-16*c1^2*c2*c3*c4+4*c1*c3^4+8*c1*c3^2*c4^2+4*c1*c4^4,...
       5*a1^2*c1^2*c4-8*a1*b1*c1^2*c4-2*b2*a1*c1^3-2*a1*c1^2*c2*c3-16*a1*c1*c3^2*c4+16*a1*c1*c4^3+3*b1^2*c1^2*c4+2*b2*b1*c1^3-2*b1*c1^2*c2*c3+20*b1*c1*c3^2*c4-12*b1*c1*c4^3+4*a2*c1^3*c2+16*c1^2*c2^2*c4+4*b2*c1^2*c3^2-8*a2*c1^2*c3*c4-4*b2*c1^2*c4^2+4*c1*c2*c3^3-60*c1*c2*c3*c4^2+12*c3^4*c4+24*c3^2*c4^3+12*c4^5,...
       a1^2*b1*c1^2+7*a1^2*c1*c4^2-2*a1*a2*c1^2*c3-2*a1*b1^2*c1^2-2*a1*b1*c1*c3^2-2*a1*b1*c1*c4^2-8*a1*b2*c1^2*c4-6*a1*c1*c2*c3*c4-20*a1*c3^2*c4^2+12*a1*c4^4+a2^2*c1^3+14*a2*c1^2*c2*c4+4*a2*c1*c3^3-28*a2*c1*c3*c4^2+b1^3*c1^2+3*b1^2*c1*c3^2-4*b1^2*c1*c4^2+6*b1*b2*c1^2*c4+4*b1*c1^2*c2^2-22*b1*c1*c2*c3*c4+36*b1*c3^2*c4^2+4*b1*c4^4+b2^2*c1^3-2*b2*c1^2*c2*c3+20*b2*c1*c3^2*c4-12*b2*c1*c4^3-3*c1*c2^2*c3^2+13*c1*c2^2*c4^2+24*c2*c3^3*c4-40*c2*c3*c4^3,...
       2*a1^2*b1*c1*c4+a1^2*b2*c1^2+3*a1^2*c4^3-a1*a2*c1^2*c2-4*a1*a2*c1*c3*c4-2*a1*b1^2*c1*c4-4*a1*b1*b2*c1^2+a1*b1*c1*c2*c3-6*a1*b1*c3^2*c4+4*a1*b1*c4^3-2*a1*b2*c1*c3^2-2*a1*b2*c1*c4^2+a1*c1*c2^2*c4-12*a1*c2*c3*c4^2+3*a2^2*c1^2*c4+3*a2*b1*c1^2*c2-6*a2*b1*c1*c3*c4+10*a2*c1*c2*c4^2+12*a2*c3^3*c4-20*a2*c3*c4^3+3*b1^2*b2*c1^2-3*b1^2*c1*c2*c3+9*b1^2*c3^2*c4+6*b1*b2*c1*c3^2-8*b1*b2*c1*c4^2+5*b1*c1*c2^2*c4-12*b1*c2*c3*c4^2+3*b2^2*c1^2*c4+4*b2*c1^2*c2^2-22*b2*c1*c2*c3*c4+36*b2*c3^2*c4^2+4*b2*c4^4-4*c1*c2^3*c3+15*c2^2*c3^2*c4-c2^2*c4^3,...
       2*a1^2*b2*c1*c4+b1*a1^2*c4^2-2*a1*a2*c1*c2*c4-2*a1*a2*c3*c4^2-2*a1*b2^2*c1^2+a1*b2*c1*c2*c3-4*b1*a1*b2*c1*c4-6*a1*b2*c3^2*c4+4*a1*b2*c4^3+b1*a1*c1*c2^2-a1*c2^2*c4^2-3*b1*a1*c2*c3*c4+3*a2^2*c1*c4^2+3*a2*b2*c1^2*c2-6*a2*b2*c1*c3*c4-3*a2*c1*c2^2*c3+3*b1*a2*c1*c2*c4+12*a2*c2*c3^2*c4-6*b1*a2*c3*c4^2+3*b1*b2^2*c1^2+3*b2^2*c1*c3^2-4*b2^2*c1*c4^2+5*b2*c1*c2^2*c4-6*b1*b2*c1*c2*c3-12*b2*c2*c3*c4^2+18*b1*b2*c3^2*c4-c1*c2^4+3*c2^3*c3*c4,...
       a1^2*b2*c4^2-a1*a2*c2*c4^2-2*a1*b2^2*c1*c4+a1*b2*c1*c2^2-3*a1*b2*c2*c3*c4+a2^2*c4^3+3*a2*b2*c1*c2*c4-6*a2*b2*c3*c4^2-a2*c1*c2^3+3*a2*c2^2*c3*c4+b2^3*c1^2-3*b2^2*c1*c2*c3+9*b2^2*c3^2*c4];
   ycoeffs = [xcoeffs(1),...
       3*a1^2*c1^2*c3-8*a1*b1*c1^2*c3+2*a2*a1*c1^3-2*a1*c1^2*c2*c4-12*a1*c1*c3^3+20*a1*c1*c3*c4^2+5*b1^2*c1^2*c3-2*a2*b1*c1^3-2*b1*c1^2*c2*c4+16*b1*c1*c3^3-16*b1*c1*c3*c4^2+4*b2*c1^3*c2+16*c1^2*c2^2*c3-4*a2*c1^2*c3^2-8*b2*c1^2*c3*c4+4*a2*c1^2*c4^2-60*c1*c2*c3^2*c4+4*c1*c2*c4^3+12*c3^5+24*c3^3*c4^2+12*c3*c4^4,...
       a1^3*c1^2-2*a1^2*b1*c1^2-4*a1^2*c1*c3^2+3*a1^2*c1*c4^2+6*a1*a2*c1^2*c3+a1*b1^2*c1^2-2*a1*b1*c1*c3^2-2*a1*b1*c1*c4^2+4*a1*c1^2*c2^2-22*a1*c1*c2*c3*c4+4*a1*c3^4+36*a1*c3^2*c4^2+a2^2*c1^3-8*a2*b1*c1^2*c3-2*a2*c1^2*c2*c4-12*a2*c1*c3^3+20*a2*c1*c3*c4^2+7*b1^2*c1*c3^2-2*b1*b2*c1^2*c4-6*b1*c1*c2*c3*c4+12*b1*c3^4-20*b1*c3^2*c4^2+b2^2*c1^3+14*b2*c1^2*c2*c3-28*b2*c1*c3^2*c4+4*b2*c1*c4^3+13*c1*c2^2*c3^2-3*c1*c2^2*c4^2-40*c2*c3^3*c4+24*c2*c3*c4^3,...
       3*a1^2*a2*c1^2-2*a1^2*b1*c1*c3-3*a1^2*c1*c2*c4+9*a1^2*c3*c4^2-4*a1*a2*b1*c1^2-8*a1*a2*c1*c3^2+6*a1*a2*c1*c4^2+2*a1*b1^2*c1*c3+a1*b1*c1*c2*c4+4*a1*b1*c3^3-6*a1*b1*c3*c4^2+3*a1*b2*c1^2*c2-6*a1*b2*c1*c3*c4+5*a1*c1*c2^2*c3-12*a1*c2*c3^2*c4+3*a2^2*c1^2*c3+a2*b1^2*c1^2-2*a2*b1*c1*c3^2-2*a2*b1*c1*c4^2+4*a2*c1^2*c2^2-22*a2*c1*c2*c3*c4+4*a2*c3^4+36*a2*c3^2*c4^2+3*b1^2*c3^3-b1*b2*c1^2*c2-4*b1*b2*c1*c3*c4+b1*c1*c2^2*c3-12*b1*c2*c3^2*c4+3*b2^2*c1^2*c3+10*b2*c1*c2*c3^2-20*b2*c3^3*c4+12*b2*c3*c4^3-4*c1*c2^3*c4-c2^2*c3^3+15*c2^2*c3*c4^2,...
       -2*a2^2*b1*c1^2+3*a1*a2^2*c1^2-4*a2^2*c1*c3^2+3*a2^2*c1*c4^2+2*a2*b1^2*c1*c3+a2*b1*c1*c2*c4-4*a1*a2*b1*c1*c3+4*a2*b1*c3^3-6*a2*b1*c3*c4^2+3*a2*b2*c1^2*c2-6*a2*b2*c1*c3*c4+5*a2*c1*c2^2*c3-6*a1*a2*c1*c2*c4-12*a2*c2*c3^2*c4+18*a1*a2*c3*c4^2+a1*b1^2*c3^2-2*b1*b2*c1*c2*c3-2*b1*b2*c3^2*c4+a1*b1*c1*c2^2-b1*c2^2*c3^2-3*a1*b1*c2*c3*c4+3*b2^2*c1*c3^2-3*b2*c1*c2^2*c4+3*a1*b2*c1*c2*c3+12*b2*c2*c3*c4^2-6*a1*b2*c3^2*c4-c1*c2^4+3*c2^3*c3*c4,...
       a2^3*c1^2-2*a2^2*b1*c1*c3-3*a2^2*c1*c2*c4+9*a2^2*c3*c4^2+a2*b1^2*c3^2+a2*b1*c1*c2^2-3*a2*b1*c2*c3*c4+3*a2*b2*c1*c2*c3-6*a2*b2*c3^2*c4-b1*b2*c2*c3^2+b2^2*c3^3-b2*c1*c2^3+3*b2*c2^2*c3*c4];
   %Compute the roots of the resultants
   rx = roots(ycoeffs);
   ry = roots(xcoeffs);
   [x,y] = meshgrid(rx(imag(rx)==0), ry(imag(ry)==0));
   %Each position is a pair of real roots
   pos = [x(:)'; y(:)'];
   pose = [0 0 0];
   error = inf;
   %Find the optimal heading angles for each position and take the pose with minimal reprojection error
   for i=1:size(pos,2)
       num = 0; denom = 0;
       for j=1:len
            xj = pos(1,i) - worldPts(1,j);
            yj = pos(2,i) - worldPts(2,j);
            sj = sin(thetas(j));
            cj = cos(thetas(j));
            num = num + xj * sj - yj * cj;
            denom = denom + xj * cj + yj * sj;
        end
        theta = -atan(num/denom);
        e1 = norm(rpError(worldPts, rotImgPts, [pos(1,i) pos(2,i) theta], beta));
        e2 = norm(rpError(worldPts, rotImgPts, [pos(1,i) pos(2,i) theta+pi], beta));
        if (e1 < error)
            pose = [pos(1,i) pos(2,i) theta];
            error = e1;
        end
        if (e2 < error)
            pose = [pos(1,i) pos(2,i) theta+pi];
            error = e2;
        end
    end
end