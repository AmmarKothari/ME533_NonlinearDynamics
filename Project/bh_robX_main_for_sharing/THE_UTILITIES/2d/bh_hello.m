

% read the XL file
hello_tab = readtable('hello.xlsx', 'Sheet','FINAL');

% plot it figure;
subplot(3,1,1); plot(hello_tab.X, hello_tab.Y,'-k'); grid('on'); axis('tight');
subplot(3,1,2); 
plot(hello_tab.X,'-bo');grid('on'); axis('tight'); xlabel('time'); ylabel('X(m)');
subplot(3,1,3); 
plot(hello_tab.Y,'or');grid('on'); axis('tight'); xlabel('time'); ylabel('Y(m)');

% plot the DIFF
hello_tab.DX = [diff(hello_tab.X); 0];
hello_tab.DY = [diff(hello_tab.Y); 0];

DT = 1/10;
hello_tab.T = DT*[0:(length(hello_tab.DX)-1)]';

hello_tab.DXDT = hello_tab.DX / DT ;
hello_tab.DYDT = hello_tab.DY / DT ;

figure;
subplot(2,1,1); 
plot(hello_tab.T, hello_tab.DXDT,'-bo');grid('on'); axis('tight'); 
 xlabel('time'); ylabel('XDOT (m/sec)');
subplot(2,1,2);  
plot(hello_tab.T, hello_tab.DYDT,'-ro');grid('on'); axis('tight'); 
 xlabel('time'); ylabel('YDOT (m/sec)');

%writetable(hello_tab, 'hello.xlsx', 'Sheet','FINAL')


