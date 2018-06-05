
thet_deg = 0:0.1:90;
x1 = 0.5*cosd(thet_deg);
y1 = 0.5*sind(thet_deg);
x2 = 1.5*cosd(thet_deg);
y2 = 1.5*sind(thet_deg);

figure;
plot(x1,y1); hold on
plot(x2,y2);
grid on
axis equal
axis([0 1.5 0 1.5])


% read the XL file
hello_tab = readtable('hello.xlsx', 'Sheet','FINAL');
plot(hello_tab.X, hello_tab.Y,'-r')