function RIRs = generate_RIR(distToWall,dimensions,distSourceToReceiv,sampFreq,absorptionRate,soundSpeed,offset)
% initialization_step;

% Generate RIR without direct-path component Y_{r} with r = 1
sourcePos=[distToWall,dimensions(2)/2,dimensions(3)/2];
receivPos=[sourcePos(1:2),sourcePos(3)-distSourceToReceiv];
% [Room,source, receiver, options] = MCroom_init(sourcePos, receivPos, init.dimensions, true,init.sampFreq,init.absorptionRate,init.soundSpeed);
% RIRd = RunMCRoomSim(source,receiver,Room,options);
[Room,source, receiver, options] = MCroom_init(sourcePos, receivPos, dimensions, false,sampFreq,absorptionRate,soundSpeed);
RIRs = RunMCRoomSim(source,receiver,Room,options);
Fs = sampFreq;
% RIRd = RIRd(init.offset:end);
RIRs = RIRs(offset:end);

        