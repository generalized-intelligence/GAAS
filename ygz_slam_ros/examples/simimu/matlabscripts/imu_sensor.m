function [ w, a ] = imu_sensor( rf, pf, tl, g )
%imu_sensor 模拟IMU传感器输出
%   根据全局坐标系下的旋转和平移轨迹计算IMU输出
%   输入 rf: 旋转轨迹
%       pf: 平移轨迹
%       tl: 时间轴
%       bias: 偏差
%       noise: 噪声
%   输出  w: 体坐标系下角速度
%        a: 体坐标系下加速度

len = length(tl);
omega = zeros(3, len);
acc = zeros(3, len);
dh = 1e-7;
% g = [0; 0; -9.81];
for i = 1:len,
    R = rf(tl(i));
    %计算R的数值导数
    R_dot = (rf(tl(i)+dh)-rf(tl(i)-dh))/(2*dh);
    %计算R的导数
    %R_dot2 = R*(skew(H(logm(rf(tl(i))))*th_dot));
    om = R'*R_dot;
    omega(:, i) = [om(3,2); om(1,3); om(2,1)];
    vf_old = (pf(tl(i))-pf(tl(i)-2*dh))/(2*dh);
    vf = (pf(tl(i)+2*dh)-pf(tl(i)))/(2*dh);
    ac = (vf-vf_old)/(2*dh);
    
    %acc(:, i) = R'* ac;
    acc(:, i) = R'* (ac-g);
end;

w = omega;
a = acc;
end

