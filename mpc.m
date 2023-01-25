 clear all
 clc
 %intialization
%car
 % s is the parameter of curve
 % s_dot =constant(c) , s_doubleDot=zero
dt=0.001; % ds=c*dt => ds=dt
s_dot=1.0;
L = 2.6;  % length of wheel base
delta_max=pi/6;
v_max=10.0;
v=10.0;
Pose = [30.0; -5.0; pi/2]; % [x y theta]
poseDot = 0*Pose;
vel=[10.0; 0.0];      %[v w]
tspan=[0 0.001];
uk=0.0;

global N_pred N_cont A P H Q R x_r_change u_r_change xk_error
N_pred=10;
N_cont=5;
Q=eye(N_pred);
R=0.25*eye(N_cont);
u_max=v/L*delta_max;
u_min=-u_max;
%u_change_min=
%u_change_max=
tolerance=1e-8; % Tolerance for optimization
oldopts = optimset ;	% Set parameters of the optimization problem
options = optimset(oldopts,'MaxIter' , 1e4, 'TolFun',tolerance,'LargeScale' , 'off' , 'TolX' , tolerance ) ;

u_r_change=zeros(N_cont,1);
u_opt=zeros(N_cont,1);
x_r_change=zeros(3*N_pred,1);
x_hat=zeros(3*N_pred,1); % Predicted state matrix
H=zeros(3*N_pred,N_cont);
P=zeros(3*N_pred,3);
xk_error=0.0;

head_err=zeros;
track_err=zeros;
ref_pose=zeros;
w_ref=zeros;
S=zeros;
pose_x=zeros;
pose_y=zeros;
f_x=zeros;
cross_track_error=zeros;



%controller
% offline
for s=0:s_dot*dt:2*pi

i=s*1000+1;
i=int32(i);
S(i)=s;
% path
% for lissajous x=a*cos(s) and y=b*sin(2s)
% v_ref= s_dot*sqrt(4*b^2+a^2-x^2-4*y^2)
% w_ref=(y_dot*x-4*x_dot*y)/(4*b^2+a^2-x^2-4*y^2)

ref_pose(1,i)= 30*cos(s); %x_ref
ref_pose(2,i)= 50*sin(2*s); %y_ref
x_dot=-30*sin(s)*s_dot;
y_dot=50*2*cos(2*s)*s_dot;
ref_pose(3,i)= atan2(y_dot,x_dot); %theta_ref
if ref_pose(3,i)<0
    ref_pose(3,i)=ref_pose(3,i)+2*pi;
end
% v_ref = s_dot*sqrt(4*50^2+30^2-x(i)^2-4*y(i)^2); 
w_ref(i)= (y_dot*ref_pose(1,i)-4*x_dot*ref_pose(2,i))/(4*50^2+30^2-ref_pose(1,i)^2-4*ref_pose(2,i)^2); %refernce input

a_k=[ 1 0 -v*dt*sin(ref_pose(3,i));
      0 1 v*dt*cos(ref_pose(3,i));
      0 0 1];

b_k=[-v*dt*sin(ref_pose(3,i));
    v*dt*cos(ref_pose(3,i));
    w_ref(i)];
A(1:3,3*i-2:3*i)=a_k(1:3,1:3);
B(1:3,i)=b_k(1:3,1);
end

N_samples=length(S);
% online
for t=0:dt:2*pi
    k=t*1000+1;
    k=int32(k);
    x(k)=ref_pose(1,k);
    y(k)=ref_pose(2,k);
    theta(k)=ref_pose(3,k);
           
if k<=N_samples-N_pred        
for i=1:N_pred
  x_r_change(3*i-2,1)=ref_pose(1, k+i)-ref_pose(1, k);
  x_r_change(3*i-1,1)=ref_pose(2, k+i)-ref_pose(2, k);
  x_r_change(3*i,1)=ref_pose(3, k+i)-ref_pose(3, k);
end 

n=N_pred;

 for j=1:3:3*N_pred-2
    P(j:j+2,1:3)=multiply(k,0,n)
 n=n-1;
%  if n==0
%      break;
%  end    
 end   
n=N_pred;
for i=1:1:N_cont
    if k==1
     u_r_change(i)=  w_ref(i+k); 
    else  
    u_r_change(i)=(w_ref(i+k-1)-w_ref(k-1)); %array not matrix
    end
    
%     if i+k>length(B)
%             break
%     end 
 
  H(3*i-2:3*i,i)=B( 1:3,i+k);

        
%   if i==N_cont-1
%      break
%   end   
  
  for j=i:3:3*N_pred-1
      H(3*i+j:3*i+j+2,i)=multiply(k,i,n)*B(1:3,i+k)
      n=n-1;
%  if n==0
%      break
%  end 
  end    
end
uk_old=uk; %saving last input
 Ukf_init =  u_opt ; %/ Initial guess for future manipulated input vector
u_opt = fmincon( 'MPC_objfn' , Ukf_init, [],[],[], [], u_max,u_min,[],options) ; % Constrained optimization problem
uk = u_opt(1,:); % Pick up only the first input move
 end

vel(1,:)=v;
vel(2,:)=w_ref(k)+uk;
Pose(1,:)=Pose(1,:) +vel(1,:)*dt*cos(Pose(3,:));
Pose(2,:)=Pose(2,:) +vel(1,:)*dt*sin(Pose(3,:));
Pose(3,:)=Pose(3,:) +vel(2,:)*dt;
if Pose(3,:)>2*pi
    Pose(3,:)=Pose(3,:)-2*pi;
end
xk_error=Pose(1:3,:)-ref_pose(1:3,k);

head_err(i)= Pose(3,:)- ref_pose(3,i);
if head_err(i) < -pi
        head_err(i) = head_err(i) + 2 * pi;
    else if head_err(i) > pi
            head_err(i) = head_err(i) - 2 * pi;
        end
    end
track_err(i)= sqrt((Pose(1,:)-ref_pose(1,i))^2+(Pose(2,:)-ref_pose(2,i))^2);
pose_x(i)=Pose(1,:);
pose_y(i)=Pose(2,:);


end

%state space model
% err_input= w_ref-vel(2,:);
% err_pose= ref_pose-Pose;
% err_pose(k+1)= A*err_pose(k) + B*err_input(k);


