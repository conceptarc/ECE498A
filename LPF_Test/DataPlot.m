csv = csvread('LPF_Test_Output_Only_Numbers_No_Aliasing.csv');
w = 2*pi*csv(:,1);
semilogx(w,csv(:,2))
title('Experimental Frequency Magnitude Plot For Low Pass Filter');
axis([-inf, inf, -inf, 5]);
ylabel('Noise Power (Db)');
xlabel('Noise Frequency (rad/s)');