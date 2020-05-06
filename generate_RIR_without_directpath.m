initialization_step;

% Generate RIR without direct-path component Y_{r} with r = 1
sourcePos=[init.distToWall,init.dimensions(2)/2,init.dimensions(3)/2];
receivPos=[sourcePos(1:2),sourcePos(3)-init.distSourceToReceiv];
% [Room,source, receiver, options] = MCroom_init(sourcePos, receivPos, init.dimensions, true,init.sampFreq,init.absorptionRate,init.soundSpeed);
% RIRd = RunMCRoomSim(source,receiver,Room,options);
[Room,source, receiver, options] = MCroom_init(sourcePos, receivPos, init.dimensions, false,init.sampFreq,init.absorptionRate,init.soundSpeed);
RIRs = RunMCRoomSim(source,receiver,Room,options);
Fs = init.sampFreq;
% RIRd = RIRd(init.offset:end);
RIRs = RIRs(init.offset:end);

        