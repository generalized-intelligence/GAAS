
clear;
close all;
clc;

[rr, pp] = traj_gen(@t3);

F = 200;
gw = [0;0;9.8];
times = 0:1/F:10;
N = length(times);
[omega, acc] = imu_sensor( rr, pp, times, gw);

wnstd = 2e-4*sqrt(F)*randn(size(omega));
anstd = 2e-3*sqrt(F)*randn(size(acc));

r = zeros(9,N);
p = zeros(3,N);
figure; hold on;
for i=1:N
    R = rr(times(i));
    r(:,i) = R(:);
    p(:,i) = pp(times(i));
    T = eye(4);
    T(1:3,1:3) = R;
    T(1:3,4) = p(:,i);
    if(mod(i,20)==0)
        plotaxis(T);
    end;
end;

ofimu = fopen('imu.txt','w');
for i=1:N
    fprintf(ofimu, '%.6f %.6f %.6f %.6f %.6f %.6f %.6f \n',...
        times(i),omega(1,i),omega(2,i),omega(3,i),...
        acc(1,i),acc(2,i),acc(3,i));
end;
fclose(ofimu);

ofrot = fopen('rot.txt','w');
for i=1:N
    fprintf(ofrot, '%.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f \n',...
        times(i),r(1,i),r(2,i),r(3,i),r(4,i),...
        r(5,i),r(6,i),r(7,i),r(8,i),r(9,i));
end;
fclose(ofrot);

ofpos = fopen('pos.txt','w');
for i=1:N
    fprintf(ofpos, '%.6f %.6f %.6f %.6f \n',...
        times(i),p(1,i),p(2,i),p(3,i));
end;
fclose(ofpos);

ofimunoise = fopen('imunoise.txt','w');
for i=1:N
    fprintf(ofimunoise, '%.6f %.6f %.6f %.6f %.6f %.6f %.6f \n',...
        times(i),wnstd(1,i),wnstd(2,i),wnstd(3,i),...
        anstd(1,i),anstd(2,i),anstd(3,i));
end;
fclose(ofimunoise);
