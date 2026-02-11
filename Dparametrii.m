clear
load PID_cls_Eln_3_90.15.mat
%%
t_start=tic;

Ts = 1;
s = tf('s');

ad = input('Introduceți valoarea pentru ad: ');
if isempty(ad)
    ad=0.1;
end

bd = input('Introduceți valoarea pentru bd: ');
if isempty(bd)
    bd=1;
end

% Afișează valorile pentru verificare
disp(['ad = ', num2str(ad)]);
disp(['bd = ', num2str(bd)]);



%%

[x,fval,Computer] = PSOD(ad,bd);

%%
Nd = 10;
Td_11 = x(1); 
Td_12 = x(2); 
Td_21 = x(3); 
Td_22 = x(4); 


Gd_11 = tf([2*Td_11*Nd, -2*Td_11*Nd], [(Nd*Ts + 2*Td_11), (Nd*Ts - 2*Td_11)], Ts);
Gd_12 = tf([2*Td_12*Nd, -2*Td_12*Nd], [(Nd*Ts + 2*Td_12), (Nd*Ts - 2*Td_12)], Ts);
Gd_21 = tf([2*Td_21*Nd, -2*Td_21*Nd], [(Nd*Ts + 2*Td_21), (Nd*Ts - 2*Td_21)], Ts);
Gd_22 = tf([2*Td_22*Nd, -2*Td_22*Nd], [(Nd*Ts + 2*Td_22), (Nd*Ts - 2*Td_22)], Ts);

Td_z_opt = [Gd_11,Gd_12; Gd_21,Gd_22]


 assignin('base', 'Td_z',Td_z_opt);
%%
simOut = sim ('PID_cls_Eln.slx');
y1 = simOut.get('y1');

Ny = length(y1);
errorsgn1 = simOut.get('eroare1');
 errorsgn2 = simOut.get('eroare2');
u1 = simOut.get('u1');
u2 = simOut.get('u2');

 J1 = norm(errorsgn1(100:end))/sqrt(Ny - 99);
J2 = norm(errorsgn2(100:end))/sqrt(Ny - 99);
J3=std(u1(100:end));
J4=std(u2(100:end));

J = 0.2*J1 + 0.2*J2+0.3*J3+0.3*J4;

ref1 = simOut.get('ref1');


ref2 = simOut.get('ref2');


u1 = simOut.get('u1');


u2 = simOut.get('u2');


y1 = simOut.get('y1');


y2 = simOut.get('y2');

figure(1)
plot(ref1);
hold on;
plot(y1);
hold off;


figure(2)
plot(ref2);
hold on;
plot(y2);
hold off;

figure (3)
plot(u1);

figure (4)
plot(u2);


% save("workspace_final.mat");
t_stop=toc(t_start); %secunde
durata_ore=t_stop/3600;
nr_ore=fix(durata_ore); % nr ore
durata_min=(durata_ore-nr_ore)*60; % in minute
nr_min=fix(durata_min);
nr_sec=fix((durata_min-nr_min)*60); % nr secunde
disp('durata simularii: ')
timp_rulare=[num2str(nr_ore) ' h:' num2str(nr_min) ' min:' num2str(nr_sec) ' s']
eval(['save PID_cls_' int2str(Computer) '.mat'])
F= 100/(1+0.65*J);

%%
eval(['save PID_cls_Eln_' int2str(Computer) '_' sprintf('%0.2f',F) '.mat']);

%%


