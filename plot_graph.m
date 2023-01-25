% subplot(3,1,1);
% plot(S,head_err,'blue');
% hold on
% plot(S,head_err2,'red');
% hold on
% title('Heading error')
% xlabel('parameter, s');
% ylabel('Heading error');
% subplot(3,1,2);
% plot(S,track_err,'blue');
% hold on
% plot(S,track_err2,'red');
% hold on
% title('Tracking error');
% xlabel('parameter, s');
% ylabel('Tracking error');
% subplot(3,1,3);
% plot(S,cross_track_error,'blue');
% hold on
% plot(S,cross_track_error2,'red');
% hold on
% title('Cross track error');
% xlabel('parameter, s');
% ylabel('Cross track error');
%  legend('using ANN model','using conventional model'); 
% 
plot(x,y,'blue');
hold on
plot(pose_x1,pose_y1,'red');
xlabel('x');
ylabel('y');
hold on
plot(pose_x2,pose_y2,'green');
hold on
% plot(pose_x3,pose_y3,'blue');
% hold on
% plot(pose_x4,pose_y4,'black');
%  hold on
legend('refernece path','using conventional model','using ANN model'); 
% 
% subplot(2,2,1);
% plot(S,v1,'red');
% xlabel('x');
% ylabel('V1');
% subplot(2,2,2);
% plot(S,v2,'green');
% subplot(2,2,3);
% plot(S,v3,'blue');
% subplot(2,2,4);
% plot(S,v4,'black');
% legend('wheel front left', 'front right', 'back left', 'back right');
% legend('wheel up', 'wheel left', 'wheel right');

% subplot(2,1,1);
% plot(S,v,'red');
% title('linear velocity, v');
% xlabel('parameter, s');
% ylabel('Linear velocity');
% subplot(2,1,2);
% plot(S,w,'red');
% title('angular velocity, w');
% xlabel('parameter, s');
% ylabel('Angular Velocity');
