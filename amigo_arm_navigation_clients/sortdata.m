close all

%% UNFILTERED

x = unfiltered(1,:);

%in case of pos , row_length = 8
%in case of pos + vel, row_length = 15
%in case of pos + vel+acc, row_length = 22

row_length = 8;

nr_points = length(x)/row_length;
pos = zeros(nr_points,7);
vel = zeros(nr_points,7);
acc = zeros(nr_points,7);
time = zeros(nr_points,1);

for i=1:nr_points
   elem = row_length *(i-1) +1; 
   pos(i,1:7) =  x(1, elem:(elem+6));
   time(i) = x(1, row_length * i); 
    
end
figure
plot(time, pos(:,1),'b')
hold
plot(time, pos(:,2),'r')
plot(time, pos(:,3),'k')
plot(time, pos(:,4),'g')
plot(time, pos(:,5),'m')
plot(time, pos(:,6),'c')
plot(time, pos(:,7),'y')
title('Unfiltered trajectory');
xlabel('seq')
ylabel('joint angles [rad]')

%% FILTERED

x = filtered(1,:);

%in case of pos , row_length = 8
%in case of pos + vel, row_length = 15
%in case of pos + vel+acc, row_length = 22

row_length = 22;

nr_points = length(x)/row_length;
pos = zeros(nr_points,7);
vel = zeros(nr_points,7);
acc = zeros(nr_points,7);
time = zeros(nr_points,1);

for i=1:nr_points
   elem = row_length *(i-1) +1; 
   pos(i,1:7) =  x(1, elem:(elem+6));
   time(i) = x(1, row_length * i); 
    
end
figure
plot(time, pos(:,1),'b')
hold
plot(time, pos(:,2),'r')
plot(time, pos(:,3),'k')
plot(time, pos(:,4),'g')
plot(time, pos(:,5),'m')
plot(time, pos(:,6),'c')
plot(time, pos(:,7),'y')
title('Filtered trajectory');
xlabel('time [s]')
ylabel('joint angles [rad]')

%% COMBINE