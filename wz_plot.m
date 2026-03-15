%% N=3
% wz=10
figure()
semilogx(logspace(-4,1,101),[PN_3(1,:);APN_3(1,:);ZMD_3(1,:)]); grid on; title("\omega_z=10(3)");
% wz=20
figure()
semilogx(logspace(-4,1,101),[PN_3(2,:);APN_3(2,:);ZMD_3(2,:)]); grid on; title("\omega_z=20(3)");
% wz=40
figure()
semilogx(logspace(-4,1,101),[PN_3(3,:);APN_3(3,:);ZMD_3(3,:)]); grid on; title("\omega_z=40(3)");
%% N=4
% wz=10
figure()
semilogx(logspace(-4,1,101),[PN_4(1,:);APN_4(1,:);ZMD_4(1,:)]); grid on; title("\omega_z=10(4)");
% wz=20
figure()
semilogx(logspace(-4,1,101),[PN_4(2,:);APN_4(2,:);ZMD_4(2,:)]); grid on; title("\omega_z=20(4)");
% wz=40
figure()
semilogx(logspace(-4,1,101),[PN_4(3,:);APN_4(3,:);ZMD_4(3,:)]); grid on; title("\omega_z=40(4)");
%% N=5
% wz=10
figure()
semilogx(logspace(-4,1,101),[PN_5(1,:);APN_5(1,:);ZMD_5(1,:)]); grid on; title("\omega_z=10(5)");
% wz=20
figure()
semilogx(logspace(-4,1,101),[PN_5(2,:);APN_5(2,:);ZMD_5(2,:)]); grid on; title("\omega_z=20(5)");
% wz=40
figure()
semilogx(logspace(-4,1,101),[PN_5(3,:);APN_5(3,:);ZMD_5(3,:)]); grid on; title("\omega_z=40(5)");