close all

%% UNFILTERED

x = unfiltered(1,:);

%in case of pos , row_length = 8
%in case of pos + vel, row_length = 15
%in case of pos + vel+acc, row_length = 22
num_joints = 7;
row_length = num_joints + 1;

nr_points = length(x)/row_length

pos = zeros(nr_points,num_joints);
vel = zeros(nr_points,num_joints);
acc = zeros(nr_points,num_joints);
time = zeros(nr_points,1);

for i=1:nr_points
   elem = row_length *(i-1) +1; 
   pos(i,1:num_joints) =  x(1, elem:(elem+(num_joints-1)));
   time(i) = x(1, row_length * i) ;
    
end


diff_q = diff(pos);

figure
subplot(2,2,1)
plot([0 :nr_points-1], pos(:,1),'b')
hold
plot([0 :nr_points-1], pos(:,2),'r')
plot([0 :nr_points-1], pos(:,3),'k')
plot([0 :nr_points-1], pos(:,4),'g')
plot([0 :nr_points-1], pos(:,5),'m')
plot([0 :nr_points-1], pos(:,6),'c')
plot([0 :nr_points-1], pos(:,num_joints),'y')
title('Unfiltered trajectory');
xlabel('seq')
ylabel('joint angles [rad]')
grid
subplot(2,2,4)
plot(pos(:,1),pos(:,2))
grid

subplot(2,2,3)
plot([0 :nr_points-2],diff_q)
grid
%% FILTERED

x = filtered(1,:);

%in case of pos , row_length = 8
%in case of pos + vel, row_length = 15
%in case of pos + vel+acc, row_length = 22

row_length = 3*num_joints + 1;
nr_points = length(x)/row_length

pos = zeros(nr_points,num_joints);
vel = zeros(nr_points,num_joints);
acc = zeros(nr_points,num_joints);
time = zeros(nr_points,1);

for i=1:nr_points
   elem_pos = row_length *(i-1) +1; 
   elem_vel = row_length *(i-1) +2; 
   elem_acc = row_length *(i-1) +3; 
   pos(i,1:num_joints) =  x(1, elem_pos:(elem_pos+(num_joints-1)));
   vel(i,1:num_joints) =  x(1, elem_vel:(elem_vel+(num_joints-1)));
   vel(i,1:num_joints) =  x(1, elem_acc:(elem_acc+(num_joints-1)));
   time(i) = x(1, row_length * i); 
    
end
subplot(2,2,2)
plot(time, pos(:,1),'b')
hold
plot(time, pos(:,2),'r')
%plot(time, pos(:,3),'k')
%plot(time, pos(:,4),'g')
%plot(time, pos(:,5),'m')
%plot(time, pos(:,6),'c')
%plot(time, pos(:,num_joints),'y')
title('Filtered trajectory');
xlabel('time [s]')
ylabel('joint angles [rad]')
grid


subplot(2,2,4)
hold on
plot(pos(:,1),pos(:,2),'g')
grid on
title('Joint space')
xlabel('q1')
ylabel('q2')



figure
plot(time, vel(:,1),'b')
hold
plot(time, vel(:,2),'r')
grid

figure
plot(time, acc(:,1),'b')
hold
plot(time, acc(:,2),'r')
grid
%% COMBINE