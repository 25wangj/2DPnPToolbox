function pose = wrapper_OPnP(camMat, camRot, worldPts, imgPts)
    [R,T,~,~] = OPnP(worldPts, inv(camMat) * imgPts);
    rot = rotm2eul(R' * camRot');
    tran = -R' * T;
    pose = [tran(1), tran(2), rot(1)];
end