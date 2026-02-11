clear
load ASTANK2_date_model.mat
%%
t_start=tic;

Ts = 1;
s = tf('s');

ap = input('Introduceți valoarea pentru ap: ');
if isempty(ap)
    ap=0.1;
end
    
bp = input('Introduceți valoarea pentru bp: ');
if isempty(bp)
    bp=1;
end
ai = input('Introduceți valoarea pentru ai: ');

if isempty(ai)
    ai=1;
end
bi = input('Introduceți valoarea pentru bi: ');
if isempty(bi)
    bi=100;
end
ad = input('Introduceți valoarea pentru ad: ');
if isempty(ad)
    ad=0.1;
end

bd = input('Introduceți valoarea pentru bd: ');
if isempty(bd)
    bd=1;
end

% Afișează valorile pentru verificare
disp(['ap = ', num2str(ap)]);
disp(['bp = ', num2str(bp)]);
disp(['ai = ', num2str(ai)]);
disp(['bi = ', num2str(bi)]);
disp(['ad = ', num2str(ad)]);
disp(['bd = ', num2str(bd)]);



%%
[x,fval,Computer] = PSO_init(ap,bp,ai,bi,ad,bd);

%%
Nd = 10;
Kp_11 = x(1);
Kp_12 = x(2);
Kp_21 = x(3);
Kp_22 = x(4);
Ti_11 = x(5);
Ti_12 = x(6);
Ti_21 = x(7);
Ti_22 = x(8);
Td_11 = x(9); 
Td_12 = x(10); 
Td_21 = x(11); 
Td_22 = x(12); 


Gi_11 = tf([Ts, Ts], [2*Ti_11, -2*Ti_11], Ts);
Gi_12 = tf([Ts, Ts], [2*Ti_12, -2*Ti_12], Ts);
Gi_21 = tf([Ts, Ts], [2*Ti_21, -2*Ti_21], Ts);
Gi_22 = tf([Ts, Ts], [2*Ti_22, -2*Ti_22], Ts);

Ti_z_opt =[Gi_11,Gi_12;Gi_21,Gi_22]

Gd_11 = tf([2*Td_11*Nd, -2*Td_11*Nd], [(Nd*Ts + 2*Td_11), (Nd*Ts - 2*Td_11)], Ts);
Gd_12 = tf([2*Td_12*Nd, -2*Td_12*Nd], [(Nd*Ts + 2*Td_12), (Nd*Ts - 2*Td_12)], Ts);
Gd_21 = tf([2*Td_21*Nd, -2*Td_21*Nd], [(Nd*Ts + 2*Td_21), (Nd*Ts - 2*Td_21)], Ts);
Gd_22 = tf([2*Td_22*Nd, -2*Td_22*Nd], [(Nd*Ts + 2*Td_22), (Nd*Ts - 2*Td_22)], Ts);

Td_z_opt = [Gd_11,Gd_12; Gd_21,Gd_22]

Kp_z_opt = [Kp_11 Kp_12; Kp_21 Kp_22]

 assignin('base', 'Ti_z',Ti_z_opt);
 assignin('base', 'Td_z',Td_z_opt);
 assignin('base', 'Kp_z',Kp_z_opt);
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

figure, fig_look(gcf,1.5,16,1) ;
plot(ref1);
hold on;
title('y1');
xlabel('Durata[s]');
ylabel('Nivelul de apa in rezervorul #1 [cm]');
plot(y1);
hold off;


figure, fig_look(gcf,1.5,16,1) ;
plot(ref2);

hold on;
title('y2');
xlabel('Durata[s]');
ylabel('Nivelul de apa in rezervorul #2 [cm]');
plot(y2);
hold off;

figure, fig_look(gcf,1.5,16,1) ;
plot(u1);
hold on;
title('u1');
xlabel('Durata[s]');
ylabel('Tensiunea electrovalvei #1 [V]');
hold off;
figure, fig_look(gcf,1.5,16,1) ;
plot(u2);
hold on;
title('u2');
xlabel('Durata[s]');
ylabel('Tensiunea electrovalvei #2 [V]');
hold off;


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


