%joint velocity limits

lim_q1 = 0.3;
lim_q2 = 0.2;
lim_q3 = 0.4;
lim_q4 = 0.35;
lim_q5 = 0.25;
lim_q6 = 0.6;
lim_q7 = 0.45;

points = 371;

seq = [0 :points]
q1 = pos(:,1).'
q2 = pos(:,2).'
q3 = pos(:,3).'
q4 = pos(:,4).'
q5 = pos(:,5).'
q6 = pos(:,6).'
q7 = pos(:,7).'






dq1 = diff(q1)
dq2 = diff(q2)
dq3 = diff(q3)
dq4 = diff(q4)
dq5 = diff(q5)
dq6 = diff(q6)
dq7 = diff(q7)





max_dq1 = max(abs(dq1));
max_dq2 = max(abs(dq2));
max_dq3 = max(abs(dq3));
max_dq4 = max(abs(dq4));
max_dq5 = max(abs(dq5));
max_dq6 = max(abs(dq6));
max_dq7 = max(abs(dq7));


r1 = lim_q1/max_dq1;
r2 = lim_q2/max_dq2;
r3 = lim_q3/max_dq3;
r4 = lim_q4/max_dq4;
r5 = lim_q5/max_dq5;
r6 = lim_q6/max_dq6;
r7 = lim_q7/max_dq7;


scale = abs(min([r1, r2, r3, r4, r5, r6, r7]))

time = seq/scale;



dq1_sc = scale * diff(q1);
dq2_sc = scale * diff(q2);
dq3_sc = scale * diff(q3);
dq4_sc = scale * diff(q4);
dq5_sc = scale * diff(q5);
dq6_sc = scale * diff(q6);
dq7_sc = scale * diff(q7);



%% plotting

figure
plot(seq,q1,'b')
hold
plot(seq,q2,'r')
plot(seq,q3,'k')
plot(seq,q4,'g')
plot(seq,q5,'m')
plot(seq,q6,'c')
plot(seq,q7,'y')
grid
title('positions')
xlabel('seq')

figure
plot(seq,[0 dq1],'b')
hold
plot(seq,[0 dq2 ],'r')
plot(seq,[0 dq3 ],'k')
plot(seq,[0 dq4 ],'g')
plot(seq,[0 dq5 ],'m')
plot(seq,[0 dq6 ],'c')
plot(seq,[0 dq7 ],'y')
grid
title('gradients')
plot(seq,lim_q1*ones(1,points+1),'b--')
plot(seq,lim_q2*ones(1,points+1),'r--')
plot(seq,lim_q3*ones(1,points+1),'k--')
plot(seq,lim_q4*ones(1,points+1),'g--')
plot(seq,lim_q5*ones(1,points+1),'m--')
plot(seq,lim_q6*ones(1,points+1),'c--')
plot(seq,lim_q7*ones(1,points+1),'y--')
xlabel('seq')

figure
plot(time,q1,'b')
hold
plot(time,q2,'r')
plot(time,q3,'k')
plot(time,q4,'g')
plot(time,q5,'m')
plot(time,q6,'c')
plot(time,q7,'y')
grid
title('positions')
xlabel('time [s]')

figure
plot(time,[0 dq1_sc],'b')
hold
plot(time,[0 dq2_sc ],'r')
plot(time,[0 dq3_sc ],'k')
plot(time,[0 dq4_sc ],'g')
plot(time,[0 dq5_sc ],'m')
plot(time,[0 dq6_sc ],'c')
plot(time,[0 dq7_sc ],'y')
grid
title('velocities')
plot(time,lim_q1*ones(1,points+1),'b--')
plot(time,lim_q2*ones(1,points+1),'r--')
plot(time,lim_q3*ones(1,points+1),'k--')
plot(time,lim_q4*ones(1,points+1),'g--')
plot(time,lim_q5*ones(1,points+1),'m--')
plot(time,lim_q6*ones(1,points+1),'c--')
plot(time,lim_q7*ones(1,points+1),'y--')
xlabel('time [s]')