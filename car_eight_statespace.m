%intialization
%car
 % s is the parameter of curve % s_dot =constant(c)= 1 , s_doubleDot=zero
% s_dot =constant(c)= 1 , s_doubleDot=zero
dt=0.001; % ds=c*dt => ds=dt
s_dot=1.0;
L = 2.6;  % length of wheel base
delta_max=pi/6;
v_max=10.0;
Pose = [30.0; -5.0; pi/2]; % [x y theta delta]
poseDot = 0*Pose;
vel=[0.0; 0.0];      %[v w]
tspan=[0 0.001];
%controller
k=[20.0; 0.1; 1.0]; % Tuning Parameters [k_x k_y k_theta] %[150,0.5,1.5]

ref_vel= [0.0; 0.0]; %[v_ref w_ref]
ref_pose=[0.0; 0.0; 0.0]; %[x_ref y_ref theta_ref 0] 
err_pose=[0.0; 0.0; 0.0]; %[x_err y_err theta_err 0]

%For plotting curve and errors
head_err=zeros;
track_err=zeros;
x=zeros;
y=zeros;
S=zeros;
pose_x=zeros;
pose_y=zeros;
v=zeros;
w=zeros;
f_x=zeros;
cross_track_error=zeros;

hold on
for s=0:s_dot*dt:2*pi

i=s*1000+1;
i=int32(i);
S(i)=s;
%path
% for lissajous x=a*cos(s) and y=b*sin(2s)
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
     0 0 1 ];
 

err_pose= T_e*(ref_pose-Pose);

%Control Law
vel(1,:)=ref_vel(1,:)*cos(err_pose(3,:)) + k(1,:)*err_pose(1,:);
vel(2,:)=ref_vel(2,:)+ ref_vel(1,:)*(k(2,:)*err_pose(2,:) + k(3,:)*sin(err_pose(3,:)));

% if(vel(1,:)<-v_max)
%     vel(1,:)=-v_max;
% end
% if(vel(1,:)>v_max)
%     vel(1,:)=v_max;
% end 
if vel(2,:)>(vel(1,:)*tan(delta_max)/L)
    vel(2,:)=vel(1,:)*tan(delta_max)/L;
end 
if vel(2,:)<(-vel(1,:)*tan(delta_max)/L)
    vel(2,:)=-vel(1,:)*tan(delta_max)/L;
end
%car

Pose(1,:)=Pose(1,:) +vel(1,:)*dt*cos(Pose(3,:));
Pose(2,:)=Pose(2,:) +vel(1,:)*dt*sin(Pose(3,:));
Pose(3,:)=Pose(3,:) +vel(2,:)*dt;
if Pose(3,:)>2*pi
    Pose(3,:)=Pose(3,:)-2*pi;
end

%error
head_err(i)= Pose(3,:)- ref_pose(3,:);
if head_err(i) < -pi
        head_err(i) = head_err(i) + 2 * pi;
    else if head_err(i) > pi
            head_err(i) = head_err(i) - 2 * pi;
        end
    end
track_err(i)= sqrt((Pose(1,:)-ref_pose(1,:))^2+(Pose(2,:)-ref_pose(2,:))^2);
pose_x(i)=Pose(1,:);
pose_y(i)=Pose(2,:);
v(i)=vel(1,:);
w(i)=vel(2,:);

% plot(pose_x(i),pose_y(i),'g*');
% drawnow;
% xlabel('x');
% ylabel('y');
% plot(x(i),y(i),'r*');
% drawnow;

n=0;
for j=s-0.05:0.001:s+0.05
X=30*cos(j);
Y= 50*sin(2*j);
n=n+1;
f_x(n)=sqrt((X-Pose(1,:))^2+(Y-Pose(2,:))^2);
  
end
cross_track_error(i)=min(f_x);
end
hold off