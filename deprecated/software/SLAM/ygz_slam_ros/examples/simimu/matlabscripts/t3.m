function y = t3( t )
%t3 
%     y = [-0.1; -0.6*t; 0.2; 1+2*t; 0; -2];
%     y = [-0.1*t; -0.8*t; 0.2; 3+2*t; 0; -2*t*t];
    y = [sin(t); sin(t); cos(t); 6*sin(t); sin(t); 3*cos(t)];
end
