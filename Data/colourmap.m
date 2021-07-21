x=meshgrid([-100:.1:100]);
y=x';
z=zeros(size(x));
z_zone=[100,75,50,10;5^2,25^2,50^2,70^2];
for i=size(z_zone,2):-1:1
    z((x.^2+y.^2)<=z_zone(2,i))=z_zone(1,i);
end
close
g1=figure
contourf(x,y,z)
b=0:.1:1;%[0 0 0 0 0 0 0 0 0 0 0]+1;
g=0:.1:1;%[0 0 0 0 0 0 0 0 0 0 0]+1;
r=[0 0 0 0 0 0 0 0 0 0 0]+1;

map = [ r(1) g(1)  b(1)
        r(2) g(2)  b(2)
        r(3) g(3)  b(3)
        r(4) g(4)  b(4)
        r(5) g(5)  b(5)
        r(6) g(6)  b(6)
        r(7) g(7)  b(7)
        r(8) g(8)  b(8)
        r(9) g(9)  b(9)
        r(10) g(10) b(10)
        r(11) g(11) b(11)]*.8;

colormap(g1,map)
axis equal

