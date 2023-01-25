%intialization
%car
 % s is the parameter of curve
 % s_dot =constant(c) , s_doubleDot=zero
dt=0.001; 
s_dot=1.0;
L = 2.6;% length of wheel base
delta_max= pi/6;
v_max=10.0;
Pose = [0.0; -5.0; atan(2)]; % [x y theta delta]
poseDot = 0*Pose;
vel=[0.0; 0.0];      %[v w]
tspan=[0 0.001];
%controller
k=[25.0; 0.4; 0.8]; % Tuning Parameters [k_x k_y k_theta]

ref_vel= [0.0; 0.0]; %[v_ref w_ref]
ref_pose=[0.0; 0.0; 0.0]; %[x_ref y_ref theta_ref 0] 
err_pose=[0.0; 0.0; 0.0]; %[x_err y_err theta_err 0]

%For plotting curve and errors
head_err2=zeros;
track_err2=zeros;
x=zeros;
y=zeros;
S=zeros;
pose_x2=zeros;
pose_y2=zeros;
v=zeros;    
w=zeros;
f_x=zeros;
cross_track_error2=zeros;

for s=0:s_dot*dt:20

i=s*1000+1;
i=int32(i);
S(i)=s;
%path
% for straight line x=a+b*s and y=c+d*s
%v_ref= s_dot*sqrt(b^2+d^2)
%w_ref=0

x(i)= s;
y(i)= 2*s;
theta= atan(2);

v_ref = s_dot*sqrt(1^2+2^2); 
w_ref= 0.0;


%controller

ref_pose(1,:) =x(i);
ref_pose(2,:) =y(i);
ref_pose(3,:) =theta;
ref_vel(1,:) = v_ref;
ref_vel(2,:) = w_ref;

T_e=[cos(Pose(3,:)) sin(Pose(3,:)) 0   %Transformation Matrix
     -sin(Pose(3,:)) cos(Pose(3,:)) 0 
     0 0 1];
 

err_pose= T_e*(ref_pose-Pose);
%Control Law
vel(1,:)= ref_vel(1,:)*cos(err_pose(3,:)) + k(1,:)*err_pose(1,:);
vel(2,:)= ref_vel(2,:)+ ref_vel(1,:)*(k(2,:)*err_pose(2,:) + k(3,:)*sin(err_pose(3,:)));

if(vel(1,:)<-v_max)
    vel(1,:)=-v_max;
end
if(vel(1,:)>v_max)
    vel(1,:)=v_max;
end    

if vel(2,:)>(vel(1,:)*tan(delta_max)/L)
    vel(2,:)=vel(1,:)*tan(delta_max)/L;
end  
if vel(2,:)<(-vel(1,:)*tan(delta_max)/L)
    vel(2,:)=-vel(1,:)*tan(delta_max)/L;
end
%car

[t,m]=ode45(@(t,m) vel(1)*cos(Pose(3)), tspan,Pose(1));
Pose(1)=m(end);
[t,m]=ode45(@(t,m) vel(1)*sin(Pose(3)), tspan,Pose(2));
Pose(2)=m(end);
[t,m]=ode45(@(t,m) vel(2), tspan,Pose(3));
Pose(3)=m(end);

%error
head_err2(i)= Pose(3,:)- ref_pose(3,:);
track_err2(i)= sqrt((Pose(1,:)-ref_pose(1,:))^2+(Pose(2,:)-ref_pose(2,:))^2);
pose_x2(i)=Pose(1,:);
pose_y2(i)=Pose(2,:);
v(i)=vel(1,:);
w(i)=vel(2,:);
n=0;
for j=s-0.05:0.001:s+0.05
X=j;
Y=2*j;
n=n+1;
f_x(n)=sqrt((X-Pose(1,:))^2+(Y-Pose(2,:))^2);
  
end
cross_track_error2(i)=min(f_x);
end
% 
% subplot(3,1,1);
% plot(x,head_err,'red');
% title('heading error')
% subplot(3,1,2);
% plot(x,track_err,'green');
% title('tracking error');
% subplot(3,1,3);
% plot(x,cross_track_error,'green');
% title('Cross track error');

% plot(pose_x,pose_y,'red');
% xlabel('x');
% ylabel('y');
% hold on
% plot(x,y);