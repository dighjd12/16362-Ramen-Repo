%% integrate figure 8 trajectory
fig_ref = figure8ReferenceControl(0.4,0.4,0.5);
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

%% feedforward testing

%feedback = boolean deciding to add feedback factor

            %need this for data logs later?
           timeArray = zeros(1,1);
           distArray = zeros(1,1); % create 1x1 matrix with value 0
           vrealArray = zeros(1,1);
           wrealArray = zeros(1,1);
           poseArray = zeros(1,1);
           pose2Array = zeros(1,1);
       
           % leftStart = double(robot.encoders.LatestMessage.Left);
           % leftEncoder = double(leftStart); % in "mm"

          %  signedDistance =0;
            arrayIndex = 1; %index starts with 1

            time=0;
         
            fig_ref = figure8ReferenceControl(0.4,0.4,0.5); 
            %fig_ref = trapezoidalStepReferenceControl(.5,.75,.25,1, 1);
            ti = 0;
            tf = fig_ref.getTrajectoryDuration(fig_ref);
            dt = .01;
            s_o = 0;
            p_o = [0; 0];
            robotTraj = robotTrajectory(ti, tf, dt, s_o, p_o, fig_ref);
            rM = robotModel();
            
            elapsedTic = tic;
            while time < tf
                pause(0.001);
                elapsedTime = toc(elapsedTic); %elapsedTime between this loop and the previous one
                elapsedTic = tic;
                time = time + elapsedTime;

                v_t = robotTraj.getVelocityAtTime(robotTraj,time);
                w_t = robotTraj.getOmegaAtTime(robotTraj,time);
               % [x;y;th] = robotTraj.getPoseAtTime(robotTraj,time);
                
               % leftEncoder = robot.encoders.LatestMessage.Left; %adds travelled distance in mm
               % signedDistance = double(double(leftEncoder - leftStart)/1000); %in m

                v_real = v_t;
                w_real = w_t;
                pose = robotTraj.getPoseAtTime(robotTraj,time);
               % [vl, vr] = rM.VwTovlvr(rM,v_real,w_real);
                pose = pose';
               % robot.sendVelocity(vl, vr);

                timeArray(arrayIndex) = time; %total elapsed time so far
              %  distArray(arrayIndex) = signedDistance; %total travelled distance so far
                vrealArray(arrayIndex) = v_real;
                wrealArray(arrayIndex) = w_real;
                poseArray(arrayIndex) = pose(1);
                pose2Array(arrayIndex) = pose(2);
                arrayIndex = arrayIndex + 1;
            end

            %robot.sendVelocity(0.0, 0.0);
            pause(1);

            figure(1);
            plot(timeArray, vrealArray);
            figure(2);
            plot(timeArray, wrealArray);
            figure(3);
            plot(poseArray, pose2Array);


%% robot stuff

robot = neato('pico');

%% robot stuff 2

robot.close()
clear all;
            
%% call trajectory follower here

ctrl = controller(0.2,0.2);

fig_ref = figure8ReferenceControl(0.4,0.4,0.5); 
%fig_ref = trapezoidalStepReferenceControl(.5,.75,.25,1, 1);

trajFollower = trajectoryFollower(ctrl, fig_ref);
trajFollower.feedForward(robot, trajFollower, true);


