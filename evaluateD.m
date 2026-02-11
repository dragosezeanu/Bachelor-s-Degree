function J = evaluateD(params)
Ts = 1;
Nd = 10;
% Parametrii RST
Td_11 = params(1); 
Td_12 = params(2); 
Td_21 = params(3); 
Td_22 = params(4); 


Gd_11 = tf([2*Td_11*Nd, -2*Td_11*Nd], [(Nd*Ts + 2*Td_11), (Nd*Ts - 2*Td_11)], Ts);
Gd_12 = tf([2*Td_12*Nd, -2*Td_12*Nd], [(Nd*Ts + 2*Td_12), (Nd*Ts - 2*Td_12)], Ts);
Gd_21 = tf([2*Td_21*Nd, -2*Td_21*Nd], [(Nd*Ts + 2*Td_21), (Nd*Ts - 2*Td_21)], Ts);
Gd_22 = tf([2*Td_22*Nd, -2*Td_22*Nd], [(Nd*Ts + 2*Td_22), (Nd*Ts - 2*Td_22)], Ts);

Td_z = [Gd_11,Gd_12; Gd_21,Gd_22];

% Setam parametrii in simulare

 
 assignin('base', 'Td_z',Td_z);


% Rulam simularea

simOut = sim ('PID_cls_Eln.slx');

 errorsgn1 = simOut.get('eroare1');
 errorsgn2 = simOut.get('eroare2');
u1 = simOut.get('u1');
u2 = simOut.get('u2');
y1 = simOut.get('y1');
 Ny=length(y1);
J1 = norm(errorsgn1(100:end))/sqrt(Ny - 99);
J2 = norm(errorsgn2(100:end))/sqrt(Ny - 99);
J3=std(u1(100:end));
J4=std(u2(100:end));
J = 0.2*J1 + 0.2*J2+0.3*J3+0.3*J4;


end




