function [src_recv_TOA, img1_TOA,img2_TOA,img3_TOA,img4_TOA] = true_TOA_est(room_dimensions,sourcePos,distSourceToReceiv,Fs)
speedofSOUND = 343;

plot_enabled = 0;

if nargin==0
    Fs = 44100;
    distSourceToReceiv=0.1;
    distToWall=0.5;
    room_dimensions=[5.4 6.38 4.05];
    sourcePos=[0+distToWall,room_dimensions(2)/2,room_dimensions(3)/2];
end

sourcePos_image_wall1=[-sourcePos(1),sourcePos(2),sourcePos(3)]; %x_wall
sourcePos_image_wall2=[sourcePos(1),-sourcePos(2),sourcePos(3)]; %y_wall
sourcePos_image_wall3=[sourcePos(1),room_dimensions(2)+(room_dimensions(2)-sourcePos(2)),sourcePos(3)]; %x_wall
sourcePos_image_wall4=[room_dimensions(1)+(room_dimensions(1)-sourcePos(1)),sourcePos(2),sourcePos(3)]; %x_wall

receivPos=[sourcePos(1:2),sourcePos(3)-distSourceToReceiv];

if plot_enabled
    
figure(10);

    % plot3([0 0 0 0],[0 dimensions(2) 0 dimensions(2)], [0 0 dimensions(3) dimensions(3)], 'r')
    plotcube(room_dimensions, [0 0 0] ,0,[1 1 0])
    hold on;

    scatter3(sourcePos(1),sourcePos(2),sourcePos(3),'r','filled')

    scatter3(receivPos(1),receivPos(2),receivPos(3),'b','filled')


    %%wall_1
    scatter3(sourcePos_image_wall1(1),sourcePos_image_wall1(2),sourcePos_image_wall1(3),'k','filled')
    plot3([sourcePos(1),sourcePos_image_wall1(1)],[sourcePos(2),sourcePos_image_wall1(2)],...
        [sourcePos(3),sourcePos_image_wall1(3)], 'r--')
    plot3([sourcePos(1),receivPos(1)],[sourcePos(2),receivPos(2)],[sourcePos(3),receivPos(3)], 'r--')
    plot3([sourcePos_image_wall1(1),receivPos(1)],[sourcePos_image_wall1(2),receivPos(2)],...
        [sourcePos_image_wall1(3),receivPos(3)], 'r--')

    %%wall_2
    scatter3(sourcePos_image_wall2(1),sourcePos_image_wall2(2),sourcePos_image_wall2(3),'k','filled')
    plot3([sourcePos(1),sourcePos_image_wall2(1)],[sourcePos(2),sourcePos_image_wall2(2)],...
        [sourcePos(3),sourcePos_image_wall2(3)], 'r--')
    plot3([sourcePos_image_wall2(1),receivPos(1)],[sourcePos_image_wall2(2),receivPos(2)],...
        [sourcePos_image_wall2(3),receivPos(3)], 'r--')

    %wall_3
    scatter3(sourcePos_image_wall3(1),sourcePos_image_wall3(2),sourcePos_image_wall3(3),'k','filled')
    plot3([sourcePos(1),sourcePos_image_wall3(1)],[sourcePos(2),sourcePos_image_wall3(2)],...
        [sourcePos(3),sourcePos_image_wall3(3)], 'r--')
    plot3([sourcePos_image_wall3(1),receivPos(1)],[sourcePos_image_wall3(2),receivPos(2)],...
        [sourcePos_image_wall3(3),receivPos(3)], 'r--')

    %wall_4
    scatter3(sourcePos_image_wall4(1),sourcePos_image_wall4(2),sourcePos_image_wall4(3),'k','filled')
    plot3([sourcePos(1),sourcePos_image_wall4(1)],[sourcePos(2),sourcePos_image_wall4(2)],...
        [sourcePos(3),sourcePos_image_wall4(3)], 'r--')
    plot3([sourcePos_image_wall4(1),receivPos(1)],[sourcePos_image_wall4(2),receivPos(2)],...
        [sourcePos_image_wall4(3),receivPos(3)], 'r--')
end
%%
%Calculation of the first-order wave TOA using image-source model

%distance measurement between image and receiver
dist_img1_src = sqrt((receivPos(1)-sourcePos_image_wall1(1)).^2 ...
+(receivPos(2)-sourcePos_image_wall1(2)).^2+(receivPos(3)-sourcePos_image_wall1(3)).^2);

dist_img2_src = sqrt((receivPos(1)-sourcePos_image_wall2(1)).^2 ...
+(receivPos(2)-sourcePos_image_wall2(2)).^2+(receivPos(3)-sourcePos_image_wall2(3)).^2);

dist_img3_src = sqrt((receivPos(1)-sourcePos_image_wall3(1)).^2 ...
+(receivPos(2)-sourcePos_image_wall3(2)).^2+(receivPos(3)-sourcePos_image_wall3(3)).^2);

dist_img4_src = sqrt((receivPos(1)-sourcePos_image_wall4(1)).^2 ...
+(receivPos(2)-sourcePos_image_wall4(2)).^2+(receivPos(3)-sourcePos_image_wall4(3)).^2);

dist_src_recv = sqrt((receivPos(1)-sourcePos(1)).^2 ...
+(receivPos(2)-sourcePos(2)).^2+(receivPos(3)-sourcePos(3)).^2);


%%
%Attenuation of the sound wave across distances
atten1 = 1/(dist_img1_src)^2;
atten2 = 1/(dist_img2_src)^2;
atten3 = 1/(dist_img3_src)^2;
atten4 = 1/(dist_img4_src)^2;

%%
%Convert the img-src distances into TOAs
img1_TOA = dist_img1_src/speedofSOUND*Fs;
img2_TOA = dist_img2_src/speedofSOUND*Fs;
img3_TOA = dist_img3_src/speedofSOUND*Fs;
img4_TOA = dist_img4_src/speedofSOUND*Fs;
src_recv_TOA = dist_src_recv/speedofSOUND*Fs;

%%
%Amplitude calculation and attenuation
src_recv_mag = 1;
img1_MAG = src_recv_mag*atten1;
img2_MAG = src_recv_mag*atten2;
img3_MAG = src_recv_mag*atten3;
img4_MAG = src_recv_mag*atten4;

%%
%Stem diagram of the 
if plot_enabled
    figure(20);
    stem(src_recv_TOA,src_recv_mag);
    hold on;
    stem(img1_TOA,1);
    stem(img2_TOA,1);
    stem(img3_TOA,1);
    stem(img4_TOA,1);
end

end