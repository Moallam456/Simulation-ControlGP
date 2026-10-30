function [out,valid] = testbench_packet(action,data)
% Private physics bridge v1. All words big-endian, doubles IEEE754 binary64.
% Command: RB2C,u32 version,sequence,enabled,f64 simTime,q[6] (72 bytes).
% Feedback: RB2F,u32 version,sequence,fault,f64 simTime,q[6],qd[6] (120 bytes).
valid=false;out=[];
switch action
    case 'decode_command'
        b=uint8(data(:).');if numel(b)~=72,return;end
        w=swapbytes(typecast(b(1:16),'uint32'));
        v=swapbytes(typecast(b(17:end),'double'));
        if w(1)~=hex2dec('52423243') || w(2)~=1 || w(4)>1 || v(1)<0 || any(~isfinite(v)),return;end
        out=struct('sequence',double(w(3)),'enabled',double(w(4)),'time',v(1),'q',v(2:7));valid=true;
    case 'encode_feedback'
        out=[typecast(swapbytes(uint32([hex2dec('52423246') 1 data.sequence data.fault])),'uint8') ...
            typecast(swapbytes(double([data.time data.q data.qd])),'uint8')];valid=true;
    case 'encode_command'
        out=[typecast(swapbytes(uint32([hex2dec('52423243') 1 data.sequence data.enabled])),'uint8') ...
            typecast(swapbytes(double([data.time data.q])),'uint8')];valid=true;
end
end
