function pose = wrapper_2DPnP(camMat, camRot, worldPts, imgPts)
    ang = rotm2eul(camRot, "ZYZ");
    rotImgPts = [cos(ang(3)) -sin(ang(3)) 0;sin(ang(3)) cos(ang(3)) 0;0 0 1] * inv(camMat) * imgPts;
    %Compute initial guess
    init = initGuess(worldPts, rotImgPts, ang(2));
    options = optimoptions("lsqnonlin", "Algorithm", "levenberg-marquardt",...
        "SpecifyObjectiveGradient", true, "Display","off");
    %Use Levenberg-Marquardt to minimize the reprojection error
    pose = lsqnonlin(@(guess)rpError(worldPts, rotImgPts, guess, ang(2)), init, [], [], options);
    pose(3) = mod(pose(3) - ang(1) + pi, 2 * pi) - pi;
end