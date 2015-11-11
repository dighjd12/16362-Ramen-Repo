
rsInWorld = pose(0.5,0.5,pi()/2);

poseInRS = pose(0.1,-0.1,pi()/2);

matpInWorld = rsInWorld.bToA() * poseInRS.bToA();

pInWorld = pose(pose.matToPoseVec(matpInWorld)).getPoseVec()


%%

startPose = [0.5;0.5;pi/2]; 
pose1 = [0.25;0.75;pi/2]; 

rsInW = pose(startPose);
rfInW = pose(pose1);
            
disp('first relative');
rfInRs = pose.matToPoseVec((rsInW.bToA()^-1) * rfInW.bToA())


pose2 = [0.75;0.25;0]; 

rsInW = pose(pose1);
rfInW = pose(pose2);
            
disp('second relative');
rfInRs = pose.matToPoseVec((rsInW.bToA()^-1) * rfInW.bToA())
pose3 = [0.5;0.5;pi/2];

rsInW = pose(pose2);
rfInW = pose(pose3);
            
disp('third relative');
rfInRs = pose.matToPoseVec((rsInW.bToA()^-1) * rfInW.bToA())





