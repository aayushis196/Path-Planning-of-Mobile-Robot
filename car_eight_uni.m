%intialization
%car
% s is the parameter of curve
% s_dot =constant(c)= 1 , s_doubleDot=zero
dt=0.001; % ds=c*dt => ds=dt
s_dot=1.0;
L = 2.6;  % length of wheel base
delta_max=pi/6;
v_max=20.0;
Pose = [30.0; -5.0; pi/2]; % [x y theta delta]
poseDot = 0*Pose;
vel=[0.0; 0.0];      %[v w]
tspan=[0 0.001];
%controller
k=[1.0; 0.05; 0.2]; % Tuning Parameters [k_x k_y k_theta]

ref_vel= [0.0; 0.0]; %[v_ref w_ref]
ref_pose=[0.0; 0.0; 0.0]; %[x_ref y_ref theta_ref 0] 
err_pose=[0.0; 0.0; 0.0]; %[x_err y_err theta_err 0]

%For plotting curve and errors
head_err=0.0;
track_err=0.0;
head_error=zeros;
track_error=zeros;
x=zeros;
y=zeros;
S=zeros;
pose_x=zeros;
pose_y=zeros;
v=zeros;
w=zeros;
f_x=zeros;
cross_track_error=zeros;

for s=0:s_dot*dt:6.28

i=s*1000+1;
i=int32(i);
S(i)=s;
%path
% for ellipse x=a*cos(s) and y=b*sin(2s)
%v_ref= s_dot*sqrt(4*b^2+a^2-x^2-4*y^2)
%w_ref=(y_dot*x-4*x_dot*y)/(4*b^2+a^2-x^2-4*y^2)

x(i)= 30*cos(s);
y(i)= 50*sin(2*s);
x_dot=-30*sin(s)*s_dot;
y_dot=50*2*cos(2*s)*s_dot;
theta= atan2(y_dot,x_dot);
if theta<0
    theta=theta+2*pi;
end
v_ref = s_dot*sqrt(4*50^2+30^2-x(i)^2-4*y(i)^2); 
w_ref= (y_dot*x(i)-4*x_dot*y(i))/(4*50^2+30^2-x(i)^2-4*y(i)^2);


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
head_err= Pose(3,:)- ref_pose(3,:);
if head_err < -pi
        head_err = head_err + 2 * pi;
    else if head_err > pi
            head_err = head_err - 2 * pi;
        end
    end
track_err= sqrt((Pose(1,:)-ref_pose(1,:))^2+(Pose(2,:)-ref_pose(2,:))^2);

%Control Law
vel(1,:)=ref_vel(1,:) + k(1,:)*track_err;
vel(2,:)=ref_vel(2,:)+ k(2,:)*head_err;

% if(vel(1,:)<-v_max)
%     vel(1,:)=-v_max;
% end
% if(vel(1,:)>v_max)
%     vel(1,:)=v_max;
% end 
% if vel(2,:)>(vel(1,:)*tan(delta_max)/L)
%     vel(2,:)=vel(1,:)*tan(delta_max)/L;
% end 
% if vel(2,:)<(-vel(1,:)*tan(delta_max)/L)
%     vel(2,:)=-vel(1,:)*tan(delta_max)/L;
% end
%car

[t,m]=ode45(@(t,m) vel(1)*cos(Pose(3)), tspan,Pose(1));
Pose(1)=m(end);
[t,m]=ode45(@(t,m) vel(1)*sin(Pose(3)), tspan,Pose(2));
Pose(2)=m(end);
[t,m]=ode45(@(t,m) vel(2), tspan,Pose(3));
Pose(3)=m(end);
if Pose(3,:)>2*pi
    Pose(3,:)=Pose(3,:)-2*pi;
end
%error
head_error(i)=head_err;
track_error(i)=track_err;
pose_x(i)=Pose(1,:);
pose_y(i)=Pose(2,:);
v(i)=ref_vel(1,:);
w(i)=ref_vel(2,:);

% n=0;
% for j=s-0.05:0.001:s+0.05
% X=30*cos(j);
% Y= 50*sin(2*j);
% n=n+1;
% f_x(n)=sqrt((X-Pose(1,:))^2+(Y-Pose(2,:))^2);
%   
% end
% cross_track_error(i)=min(f_x);
end
% subplot(2,1,1);
% plot(S,head_error,'red');
% title('heading error')
% subplot(2,1,2);
% plot(S,track_error,'green');
% title('tracking error');


plot(pose_x,pose_y,'green');
xlabel('x');
ylabel('y');
hold on
plot(x,y);