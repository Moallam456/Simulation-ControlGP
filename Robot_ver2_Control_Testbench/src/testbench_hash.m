function value=testbench_hash(file)
f=fopen(file,'rb');assert(f>=0,'testbench:HashRead','Cannot read source file.');c=onCleanup(@() fclose(f));
digest=java.security.MessageDigest.getInstance('SHA-256');digest.update(fread(f,Inf,'*uint8'));
value=lower(reshape(dec2hex(typecast(digest.digest(),'uint8'),2).',1,[]));
end
