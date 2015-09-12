%irToxy(i,r) 
function [ x, y, th] = irToXy( i, r )
th = i*(pi/180);
x = r*cos(th);
y = r*sin(th);
end