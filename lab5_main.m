%% integrate figure 8 trajectory
fig_ref = figure8ReferenceControl(0.5,0.4,0.5);
dt = 0.1;
t = 0;
tf = fig_ref.getTrajectoryDuration(fig_ref);
xArray = zeros(1,1);
yArray = zeros(1,1);
thArray = zeros(1,1);
index = 2;
while t < tf
    t = t + dt;
    [V,w] = fig_ref.computeControl(fig_ref,t);
    disp_th = double(w * dt);
    thArray(index) = thArray(end) + disp_th;
    th = thArray(index-1);
    dist = V * dt;
    disp_y = dist * sin(th);
    disp_x = dist * cos(th);
    xArray(index) = xArray(end) + disp_x;
    yArray(index) = yArray(end) + disp_y;
    index = index + 1;
end
plot(xArray,yArray);

