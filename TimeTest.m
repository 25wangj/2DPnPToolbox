funcs = {@wrapper_EPnP, @wrapper_OPnP, @wrapper_LHM, @wrapper_MLPnP, @wrapper_2DPnP};    
styles = ["-r.", "-bx", "-co", "-g*", "-k^"];
tests = 250;
pts = 50:50:1000;
camMat = [800 0 0;0 800 0;0 0 1];   
data = zeros(length(funcs), length(pts));
close all;
for i=1:length(pts)
    rots = zeros(3, 3, tests);  
    points = rand(3, pts(i), tests);
    points(1,:,:) = points(1,:,:) * 4 - 2;
    points(2,:,:) = points(2,:,:) * 4 - 2;
    points(3,:,:) = points(3,:,:) * 4 + 4;
    homPts = pagemtimes(camMat, points);
    camPts = homPts ./ homPts(3,:,:) + cat(1, 2 * randn(2, pts(i), tests), zeros(1, pts(i), tests));
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
        tic();
        for k=1:tests
            func(camMat, rots(:,:,k), points(:,:,k), camPts(:,:,k));
        end
        data(j,i) = toc() * 1000;
        disp(pts(i) + " " + func2str(func));
    end 
end
data = data / tests;
save("Data/TimeData.mat", "data");
figure(1);
hold on;
for i=1:length(funcs)
    plot(pts, data(i,:), styles(i), "MarkerSize", 3);
end
axis([0 1000 0 50])
fontsize(scale=0.5);
title("Time");
xlabel("Number of Points");
ylabel("Average Runtime (ms)");
pbaspect([3 1 1]);
print("Plots/TimePlot.eps", "-depsc2");