function pose = wrapper_LHM(camMat, camRot, worldPts, imgPts)
    normImgPts = inv(camMat) * imgPts; 
    options.method = "SVD";
    [R,T,~,~,~] = objpose(worldPts, normImgPts(1:2,:), options);
    rot = rotm2eul(R' * camRot');
    tran = -R' * T;
    pose = [tran(1), tran(2), rot(1)];
end