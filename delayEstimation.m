function [delayEst,costFunction] = delayEstimation(signal_reflectionsFft,signal_cleanFft,delayCand)

costFunction = zeros(length(delayCand),1);
for dd=1:length(delayCand)
    tmpDelay=delayCand(dd);        

%     tmpResponse=findFirFilterResponse(tmpGain,tmpDelay,length(signals.cleanFft));

    w=(0:(length(signal_cleanFft)-1))/length(signal_cleanFft)*2*pi;
    z=zeros(length(w),1);
    for ww=1:length(w)
        if w(ww)<=pi
            z(ww,1)=exp(1i*w(ww)*-tmpDelay);
        else
            tmp=w(ww)-2*pi;
            z(ww,1)=exp(1i*tmp*-tmpDelay);
        end
    end
    Xz=signal_cleanFft.*z; %Z(Tw)*S
%     Yb=signalReceivedFft-signalCleanFft.*oldResponse; % Y - Sw*Z(Tw)
%     gainEst=(Yb'*Xz+Xz'*Yb)/(2*Xz'*Xz);
    costFunction(dd,1) = (real((signal_reflectionsFft'*Xz)));
    
%             signals.cleanFft.*(tmpResponse+oldResponse)).^2);
%     end
end

%     gainEst=gainCand;
[~,delayNdx]=max(costFunction);
delayEst=delayCand(delayNdx);
end