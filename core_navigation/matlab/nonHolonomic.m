function [insAtt_new,insVel_new,insLLH_new,x_err_new,P_new]= nonHolonomic(insVel_old,insAtt_old,insLLH_old,x_err_old,P_old,omega_n_ie,omega_b_ib,A)

Cn2bUnc = eulr2dcm(insAtt_old);
%% Measurement Matrix members --
H32=-[0,1,0]*Cn2bUnc;
H42=-[0,0,1]*Cn2bUnc;
%% Measurement Innovation w/o time -- INS eq:16.42 --
z31=-[0,1,0]*(Cn2bUnc*insVel_old- skewsymm(omega_b_ib)*[-A,0,0]');
z41=-[0,0,1]*(Cn2bUnc*insVel_old- skewsymm(omega_b_ib)*[-A,0,0]');
%% Measurement Matrix -- Checked
H=[zeros(1,3), H32, zeros(1,3), zeros(1,3) zeros(1,3);
   zeros(1,3), H42, zeros(1,3), zeros(1,3) zeros(1,3)];
%% Measurement Noise Covariance Matrix
R= [0.05,0;
    0,0.05];
%% Kalman Gain
K=P_old*H'*inv(H*P_old*H'+R);

%% Update state estimation
x_err_new=(x_err_old+K*([z31;z41]-H*x_err_old));%
insAtt_new=dcm2eulr((eye(3)-skewsymm(x_err_new(1:3)))*Cn2bUnc');
insVel_new=(insVel_old-x_err_new(4:6));
insLLH_new=insLLH_old-x_err_new(7:9);
x_err_new(1:9)=zeros(1,9);
%% Update P matrix
P_new=(eye(15) - K*H)*P_old*(eye(15)-K*H)' + K*R*K';

end