%% calculating the position, velocity, acceleration from Left Foot Heel as ref.
% T_RA = joint_angle(1) //  T_LA = joint_angle(2) T_RT = joint_angle(3) //
% T_LT = joint_angle(4) RT_RS= joint_angle(5) //  LT_LS= joint_angle(6)
% RS_RF= joint_angle(7) //  LS_LF= joint_angle(8)
% ****** Left body in red and right body in green **********
function [res] =  RFB(JointAngles)
prompt = {'Enter a value of starting frame: ','Enter a value of ending frame: '};
dlgtitle = 'Enter Frames to Simulate';
dims = [1 60];
definput = {'1','445'};
answer = inputdlg(prompt,dlgtitle,dims,definput);
answer = str2double(answer);
joints =JointAngles(answer(1):answer(2),:);
res = zeros (size(joints,1),2,17);
for i = 1:length(joints)
    phi = joints(i,2:size(joints,2));
    org_RFH= [0;0];
    org_RF = [14;12.1]; % R Foot 
    org_RFT= org_RF + [58;-12.1];
    RF_com = org_RF + [16.7;-2.4];
    RS_com = org_RF + (Rot(-phi(7))*[1.1;87.2]);
    org_RS = org_RF + (Rot(-phi(7))*[0;123]); % R Shank
    RT_com = org_RS + (Rot(-phi(7))*Rot(-phi(5))*[2.3;83.5]);
    org_RT = org_RS + (Rot(-phi(7))*Rot(-phi(5))*[0;122.5]); % R Thigh
    T_com  = org_RT + (Rot(-phi(7))*Rot(-phi(5))*Rot(-phi(3))*[0;88.4]);
    org_T  = org_RT + (Rot(-phi(7))*Rot(-phi(5))*Rot(-phi(3))*[0;144]);
    org_LA = org_T  + (Rot(-phi(7))*Rot(-phi(5))*Rot(-phi(3))*Rot(phi(2))*[0;-110]);
    LA_com = org_T  + (Rot(-phi(7))*Rot(-phi(5))*Rot(-phi(3))*Rot(phi(2))*[0;-55]);
    org_RA = org_T  + (Rot(-phi(7))*Rot(-phi(5))*Rot(-phi(3))*Rot(-phi(1))*[0;-110]);
    RA_com = org_T  + (Rot(-phi(7))*Rot(-phi(5))*Rot(-phi(3))*Rot(-phi(1))*[0;-55]);
    LT_com = org_RT + (Rot(-phi(7))*Rot(-phi(5))*Rot(-phi(3))*Rot(phi(4))*[2.3;-39]);
    org_LT = org_RT + (Rot(-phi(7))*Rot(-phi(5))*Rot(-phi(3))*Rot(phi(4))*[0;-122.5]);
    LS_com = org_LT + (Rot(-phi(7))*Rot(-phi(5))*Rot(-phi(3))*Rot(phi(4))*Rot(phi(6))*[1.1;-35.8]);
    org_LS = org_LT + (Rot(-phi(7))*Rot(-phi(5))*Rot(-phi(3))*Rot(phi(4))*Rot(phi(6))*[0;-123]);
    org_LFH = org_LS+ (Rot(-phi(7))*Rot(-phi(5))*Rot(-phi(3))*Rot(phi(4))*Rot(phi(6))*Rot(phi(8))*[-14;-12.1]);
    org_LFT = org_LS+ (Rot(-phi(7))*Rot(-phi(5))*Rot(-phi(3))*Rot(phi(4))*Rot(phi(6))*Rot(phi(8))*[58;-12.1]);
    LF_com  = org_LS+ (Rot(-phi(7))*Rot(-phi(5))*Rot(-phi(3))*Rot(phi(4))*Rot(phi(6))*Rot(phi(8))*[16.7;-2.4]);
    res(i,:,:) = [org_RS org_RT org_T org_LA org_RA org_LT org_LS org_LFH org_LFT RS_com RT_com T_com LA_com RA_com LT_com LS_com LF_com];
end
p_val = zeros(size(res,1),34);
for t = 1:size(res,1)
    % we have used squeeze function here to remove one side 1 dimention and after that we have only two dimentions
    p_val(t,:) = reshape(squeeze(res(t,:,:)),1,[]); % convert the results from res to 1x34
    hold on
    %%Robot Plotting
    %plotting right Foot
    plot([org_RFH(1)  org_RF(1)],[org_RFH(2) org_RF(2)],'Color','g');
    plot([org_RF(1)  org_RFT(1)],[org_RF(2) org_RFT(2)],'Color','g');
    plot([org_RFH(1) org_RFT(1)],[org_RFH(2) org_RFT(2)],'Color','g');
    %plotting Robot Segments 
    plot([org_RF(1)  p_val(t,1)],[org_RF(2) p_val(t,2)],'Color','g'); % right Shank
    plot([p_val(t,1) p_val(t,3)],[p_val(t,2) p_val(t,4)],'Color','g'); % right Thigh
    plot([p_val(t,3) p_val(t,5)],[p_val(t,4) p_val(t,6)],'Color','b'); % Trunk
    plot([p_val(t,5) p_val(t,7)],[p_val(t,6) p_val(t,8)],'Color','r'); % Left Arm
    plot([p_val(t,5) p_val(t,9)],[p_val(t,6) p_val(t,10)],'Color','g'); % Right Arm
    plot([p_val(t,3) p_val(t,11)],[p_val(t,4) p_val(t,12)],'Color','r'); % Left Thigh
    plot([p_val(t,11) p_val(t,13)],[p_val(t,12) p_val(t,14)],'Color','r'); % left shank
    plot([p_val(t,13) p_val(t,15)],[p_val(t,14) p_val(t,16)],'Color','r'); % Left Foot heel
    plot([p_val(t,15) p_val(t,17)],[p_val(t,16) p_val(t,18)],'Color','r'); % Left Foot toe
    plot([p_val(t,13) p_val(t,17)],[p_val(t,14) p_val(t,18)],'Color','r'); % Left Right foot toe to ankel
    %%center of mass
    plot(RF_com(1),RF_com(2),'*');
    plot(p_val(t,19),p_val(t,20),'*');
    plot(p_val(t,21),p_val(t,22),'*');
    plot(p_val(t,23),p_val(t,24),'*');
    plot(p_val(t,25),p_val(t,26),'*');
    plot(p_val(t,27),p_val(t,28),'*');
    plot(p_val(t,29),p_val(t,30),'*');
    plot(p_val(t,31),p_val(t,32),'*');
    plot(p_val(t,33),p_val(t,34),'*');
    axis equal;
    pause(1/100);
    clf;
end
% getting the x and y postion of the characteristic points.
x_pos_RS = squeeze (res(:,1,1)); % LS
y_pos_RS = squeeze (res(:,2,1));
x_pos_RT = squeeze (res(:,1,2)); %LT
y_pos_RT = squeeze (res(:,2,2));
new_RF = [(org_RF(1) - x_pos_RS),(org_RF(2) - y_pos_RS)];
new_RT = [(x_pos_RT - x_pos_RS ),(y_pos_RT - y_pos_RS)];
theta1_R   = abs(atand(new_RT(:,2)./new_RT(:,1)));
theta2_R  = abs(atand(new_RF(:,2)./new_RF(:,1)));
R_knee_angle = theta1_R + theta2_R;
x_pos_LS = squeeze (res(:,1,7)); % RS
y_pos_LS = squeeze (res(:,2,7));
x_pos_LT = squeeze (res(:,1,6)); %RT
y_pos_LT = squeeze (res(:,2,6));
new_RT_L = [(x_pos_RT - x_pos_LT),(y_pos_RT - y_pos_LT)];
new_LS = [(x_pos_LS - x_pos_LT ),(y_pos_LS - y_pos_LT)];
theta1_L   = abs(atand(new_RT_L(:,2)./new_RT_L(:,1)));
theta2_L = abs(atand(new_LS(:,2)./new_LS(:,1)));
L_knee_angle = theta1_L + theta2_L;
knee_angle = L_knee_angle - R_knee_angle;
frames = linspace(1,445,445);
subplot(3,1,1);
plot(frames,knee_angle,"Color",'g');
hold on;
grid on;
plot([0,445],[3,3],'--');
plot([0,445],[-15,-15],'--');
xt = [150 300 300];
yt = [-5 20 -30];
str = {'Doble Support Phase','Single Support Phase','Single Support Phase'};
text(xt,yt,str)
xlabel('frames');
ylabel('Differance between Knee angles');
title("Right foot Base Analysis");
hold off;
% get the frames where we only have single support phase.
x = zeros(size(knee_angle,1),1);
angle = zeros(size(knee_angle,1),1);
for i = 1:size(knee_angle,1)
    x(i,:) = knee_angle(i,:);
    if (-16 < x(i,:))&&(x(i,:) < 3) % by observation the margin for the ZMP 
        angle(i,:) = x(i,:);
    end
end
% calcualting the ZMP
single_frames = (find(angle==0));
single_frames = transpose(single_frames(1:381,:)); 
ZMP_x = zeros((size(single_frames,2)),1);
ZMP_xf = zeros((size(single_frames,2)),1);
pos_x = squeeze (res(:,1,10:17)); % selecting only center of masses 
pos_x = [repmat(RF_com(1),size(res,1),1) pos_x];
fc = 10; % cutoff frequency [Hz]
fs = 1/0.01; % sampling Frequency (Framerate) [Hz].
n_order = 4; % order of the filter to discribe the steepness. 
[b,a] = butter(n_order,fc/(fs/2)); % create the filter coefficients.
pos_xfilt = filtfilt(b,a,pos_x); % filtering the data    
pos_x = transpose(pos_x);
pos_xfilt = transpose(pos_xfilt); %% fitlerd data
pos_y = squeeze (res(:,2,10:17));
pos_y = [repmat(RF_com(2),size(res,1),1) pos_y];
pos_yfilt = filtfilt(b,a,pos_y); %filtereing the data
pos_y = transpose(pos_y);
pos_yfilt = transpose(pos_yfilt);
M = [0.149 0.192 0.241 0.696 0.091 0.091 0.241 0.192 0.149];
T = 0.01; % this is Delta T(Time between each frame)
acc_x = zeros(size(pos_x,1),size(pos_x,2)-2);
acc_y = zeros(size(pos_y,1),size(pos_y,2)-2);
acc_x_filt = zeros(size(pos_xfilt,1),size(pos_xfilt,2)-2);
acc_y_filt = zeros(size(pos_yfilt,1),size(pos_yfilt,2)-2);
for j = 1:size(pos_x,1)
    acc_x(j,:) = diff(pos_x(j,:),2)./T; % this exactly like diff(diff) to have acceleration
    acc_x_filt(j,:) = diff(pos_xfilt(j,:),2)./T;
end
for k = 1:size(pos_y,1)
    acc_y(k,:) = diff(pos_y(k,:),2)./T;
    acc_y_filt(k,:) = diff(pos_yfilt(k,:),2)./T;
end
acc_y = transpose(acc_y);
acc_y_filt = transpose(acc_y_filt);
Dnume = M.*(acc_y - 9810);
Dnumef = M.*(acc_y_filt - 9810);
acc_x = transpose(acc_x);
acc_x_filt = transpose(acc_x_filt);
pos_x = transpose(pos_x(:,3:size(res,1)));
pos_xfilt = transpose(pos_xfilt(:,3:size(res,1)));
pos_y = transpose(pos_y(:,3:size(res,1)));
pos_yfilt = transpose(pos_yfilt(:,3:size(res,1)));
Nume_2= M.*acc_x .*pos_y;
Nume_2f = M.*acc_x_filt .*pos_yfilt;
Nume_1= pos_x.* Dnume;
Nume_1f = pos_xfilt.* Dnumef;
for z = 1:(size(single_frames,2)) % here we will be according to frame single phases
    f = single_frames(1,z);
    ZMP_x(z,:) = (sum(Nume_1(f,:)) - sum(Nume_2(f,:)))./sum(Dnume(f,:));
    ZMP_xf(z,:)=(sum(Nume_1f(f,:)) - sum(Nume_2f(f,:)))./sum(Dnumef(f,:));
end
subplot(3,1,2);
plot(single_frames,-ZMP_x);
hold on 
grid on;
plot([0,445],[72,72],'--');
plot([0,445],[0,0],'--');
xt = [150 300 250];
yt = [35 -50 90];
str = {'ZMP','FZMP','FZMP'};
text(xt,yt,str)
ylabel('ZMP in [mm]');
xlabel('single frames');
subplot(3,1,3);
plot(single_frames,-ZMP_xf);
hold on 
grid on;
plot([0,445],[72,72],'--');
plot([0,445],[0,0],'--');
xt = [150 300 250];
yt = [35 -50 90];
str = {'ZMP','FZMP','FZMP'};
text(xt,yt,str)
ylabel('ZMP with filterd data [mm]');
xlabel('single Frames');
end
% Rotation Function
function R = Rot(theta)
R = [cos(theta) -sin(theta);sin(theta) cos(theta)];
end