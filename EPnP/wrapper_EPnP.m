function pose = wrapper_EPnP(camMat, camRot, worldPts, imgPts)
    len = size(worldPts, 2);
    [R,T,~,~] = efficient_pnp_gauss([worldPts' ones(len)], imgPts', camMat);
    rot = rotm2eul(R' * camRot');
    tran = -R' * T;
    pose = [tran(1), tran(2), rot(1)];
end