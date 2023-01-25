%intialization
%car
 % s is the parameter of curve
 % s_dot =constant(c)= 1 , s_doubleDot=zero
dt=0.001; % ds=c*dt => ds=dt
L = 0.2;  % length of wheel base
Pose = [0.0; 0.0; 0.0; 0.0]; % [x y theta delta]
poseDot = 0*Pose;
vel=[0.0; 0.0];      %[v w]

%controller
k=[1.0; 1.0; 1.0]; % Tuning Parameters [k_x k_y k_theta]

ref_vel= [0.0; 0.0]; %[v_ref w_ref]
ref_pose=[0.0; 0.0; 0.0; 0.0]; %[x_ref y_ref theta_ref 0] 
err_pose=[0.0; 0.0; 0.0; 0.0]; %[x_err y_err theta_err 0]

%For plotting curve and errors
head_err=zeros;
track_err=zeros;
x=zeros;
y=zeros;
pose_x=zeros;
pose_y=zeros;

for s=0:0.001:10

i=s*1000+1;
i=int32(i);
%path
% for straight line x=a+b*s and y=c+d*s
%v_ref= s_dot*sqrt(b^2+d^2)
%w_ref=0

x(i)= s;
y(i)= 2*s;
theta= atan(2);

v_ref = 1.0*sqrt(1^2+2^2); 
w_ref= 0.0;


%controller

ref_pose(1,:) =x(i);
ref_pose(2,:) =y(i);
ref_pose(3,:) =theta;
ref_vel(1,:) = v_ref;
ref_vel(2,:) = w_ref;

T_e=[cos(Pose(3,:)) sin(Pose(3,:)) 0 0  %Transformation Matrix
     -sin(Pose(3,:)) cos(Pose(3,:)) 0 0
     0 0 1 0
     0 0 0 0];
 

err_pose= T_e*(ref_pose-Pose);
%Control Law
vel(1,:)= ref_vel(1,:)*cos(err_pose(3,:)) + k(1,:)*err_pose(1,:);
vel(2,:)= ref_vel(2,:)+ ref_vel(1,:)*(k(2,:)*err_pose(2,:) + k(3,:)*sin(err_pose(3,:)));

%car

poseDot(1,:) = vel(1,:)*cos(Pose(3,:));
poseDot(2,:) = vel(1,:)*sin(Pose(3,:));
poseDot(3,:) = (vel(1,:)/L)*tan(Pose(4,:));
poseDot(4,:) = vel(2,:);  
            
Pose = Pose + dt*poseDot;

%error
head_err(i)= Pose(3,:)- ref_pose(3,:);
track_err(i)= sqrt((Pose(1,:)-ref_pose(1,:))^2+(Pose(2,:)-ref_pose(2,:))^2);
pose_x(i)=Pose(1,:);
pose_y(i)=Pose(2,:);
end
% plot(x,head_err,'red');
% hold
% plot(x,track_err,'green');
plot(x,y);
hold
plot(pose_x,pose_y,'red');
hold
