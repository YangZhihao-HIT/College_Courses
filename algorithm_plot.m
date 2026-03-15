%% N=3
% PN
figure()
semilogx(logspace(-4,1,101),PN_3); grid on; title("PN(3)");
%APN
figure()
semilogx(logspace(-4,1,101),APN_3); grid on; title("APN(3)");
figure()
%ZMD-PN
semilogx(logspace(-4,1,101),ZMD_3); grid on; title("ZMD-PN(3)");
%% N=4
% PN
figure()
semilogx(logspace(-4,1,101),PN_4); grid on; title("PN(4)");
%APN
figure()
semilogx(logspace(-4,1,101),APN_4); grid on; title("APN(4)");
figure()
%ZMD-PN
semilogx(logspace(-4,1,101),ZMD_4); grid on; title("ZMD-PN(4)");
%% N=5
% PN
figure()
semilogx(logspace(-4,1,101),PN_5); grid on; title("PN(5)");
%APN
figure()
semilogx(logspace(-4,1,101),APN_5); grid on; title("APN(5)");
figure()
%ZMD-PN
semilogx(logspace(-4,1,101),ZMD_5); grid on; title("ZMD-PN(5)");