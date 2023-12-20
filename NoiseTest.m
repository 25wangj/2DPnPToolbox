funcs = {@wrapper_EPnP, @wrapper_OPnP, @wrapper_LHM, @wrapper_MLPnP, @wrapper_2DPnP};    
styles = ["-r.", "-bx", "-co", "-g*", "-k^"];
tests = 250;
pts = 50;
noise = 1:10;
camMat = [800 0 0;0 800 0;0 0 1];   
data = zeros(2, length(funcs), length(noise));
close all;
for i=1:length(noise)
    rots = zeros(3, 3, tests);  
    points = rand(3, pts, tests);
    points(1,:,:) = points(1,:,:) * 4 - 2;
    points(2,:,:) = points(2,:,:) * 4 - 2;
    points(3,:,:) = points(3,:,:) * 4 + 4;
    homPts = pagemtimes(camMat, points);
    camPts = homPts ./ homPts(3,:,:) + cat(1, noise(i) * randn(2, pts, tests), zeros(1, pts, tests));
    for j=1:tests
        rot = eye(3);
        while abs(rot(3,3)) > cos(pi/18)
            rot = quat2rotm(randrot());
        end
        points(:,:,j) = rot * points(:,:,j);
        rots(:,:,j) = rot;
    end
    for j=1:length(funcs)
        func = funcs{j};
        for k=1:tests
            pose = func(camMat, rots(:,:,k), points(:,:,k), camPts(:,:,k));
            data(1,j,i) = data(1,j,i) + norm(pose(1:2));
            data(2,j,i) = data(2,j,i) + 180 / pi * abs(pose(3));
        end
        disp(noise(i) + " " + func2str(func));
    end 
end
data = data / tests;
save("Data/NoiseData.mat", "data");
figure(1);
hold on;
for i=1:length(funcs)
    plot(noise, squeeze(data(1,i,:)), styles(i));
end
axis([1 10 0 0.5]);
title("Translational Error and Noise");
xlabel("Gaussian Noise (px)");
ylabel("Average Translational Error (m)");
pbaspect([1.5 1 1]);
print("Plots/NoiseTranslationPlot.eps", "-depsc2");
figure(2);
hold on;
for i=1:length(funcs)
    plot(noise, squeeze(data(2,i,:)), styles(i))
end
axis([1 10 0 3]);
title("Rotational Error and Noise");
xlabel("Gaussian Noise (px)");
ylabel("Average Rotational Error (deg)");
pbaspect([1.5 1 1]);
print("Plots/NoiseRotationPlot.eps", "-depsc2");