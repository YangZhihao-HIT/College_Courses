clc
a_Tmax=200;
a_cmax=600;
N=5;
intercept_time = 10;
tau = 0.3;
zeta = 0.5;
w_n = 10;
v_cl = 1000;
Ts_guidance = 0.001;
h=conv(conv([0.1 1],[0.1 1]),[0.1 1]);
Mode=2;
figure()
history=[];
APN_5=[];
for w_z=[10 20 40]
	for w_T=logspace(-4,1,101)
		sim Frequency_Domain_Corrected_PN_Guidance
		history=[history, simout(end)];
	end
	semilogx(logspace(-4,1,101), history)
	APN_5=[APN_5; history];
	hold on
	legend
	history=[];
end