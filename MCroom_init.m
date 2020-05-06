function [Room,source, receiver, options] = MCroom_init(sourcePOS,receiverPOS, room_dim, directComp,sampFreq,wallabsorption,soundSpeed)
if nargin <= 1
    error('sourcePOS, receiverPOS, room_dim not defined.')
end


Room = SetupRoom('Dim',room_dim, 'Absorption',wallabsorption*ones(6,6));
source = AddSource([],'Type','omnidirectional','Location',sourcePOS);
receiver = AddReceiver([],'Type','omnidirectional','Location',receiverPOS);
% source = AddSource([],'Type','omnidirectional','Location',sourcePOS,'Fs',sampFreq);
% receiver = AddReceiver([],'Type','omnidirectional','Location',receiverPOS,'Fs',sampFreq);
% receiver = AddReceiver([],'Type','omnidirectional','Location',receiverPOS, 'Orientation',[180,0,0],'Fs',sampFreq);
if directComp ==1
    options = MCRoomSimOptions('Fs', sampFreq, 'SimDirect',directComp,'SoundSpeed',soundSpeed, 'Duration',0.5, 'AutoCrop',false, 'SimDiff',false);
%     options = MCRoomSimOptions('SimDirect',true,'Fs',sampFreq,'Duration',reverberationTime,'FlipPhase',false,'SimDiff',false,'SoundSpeed',343,'SimSpec',true);
else
    options = MCRoomSimOptions('Fs', sampFreq, 'SimDirect',directComp,'SoundSpeed',soundSpeed,'Duration',0.5,'AutoCrop',false, 'SimDiff',false); 
%     options = MCRoomSimOptions('SimDirect',false,'Fs',sampFreq,'Duration',reverberationTime,'FlipPhase',false,'SimDiff',false,'SoundSpeed',343,'SimSpec',true); 
end

end