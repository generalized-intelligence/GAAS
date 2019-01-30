function plotaxis(Twi)
% plot frame axis of pose Twi
% Twi: pose 
hold on; xlabel('x');ylabel('y');zlabel('z');
Rwi = Twi(1:3,1:3);
twi = Twi(1:3,4);

oi = twi;
axisx = Rwi*[1;0;0];
plot3([oi(1),oi(1)+axisx(1)],[oi(2),oi(2)+axisx(2)],[oi(3),oi(3)+axisx(3)],'r-');
axisy = Rwi*[0;1;0];
plot3([oi(1),oi(1)+axisy(1)],[oi(2),oi(2)+axisy(2)],[oi(3),oi(3)+axisy(3)],'g-');
axisz = Rwi*[0;0;1];
plot3([oi(1),oi(1)+axisz(1)],[oi(2),oi(2)+axisz(2)],[oi(3),oi(3)+axisz(3)],'b-');

end