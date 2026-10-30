"""Linux integration: real SIL Kit + six AVATAR nodes + deliberately lagged physics fixture.
This tests the bridge, NOT MATLAB dynamics nor the production Supervisor.
"""
import argparse
from pathlib import Path
import socket
import struct
import subprocess
import threading

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--build',required=True,type=Path)
    parser.add_argument('--silkit',required=True,type=Path)
    args=parser.parse_args()
    stop=threading.Event()
    server=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    server.bind(('127.0.0.1',5006));server.settimeout(.1)
    received=[]
    def physics():
        time=0.
        while not stop.is_set():
            try:
                data,_=server.recvfrom(1024)
            except socket.timeout:
                continue
            assert len(data)==72
            magic,version,seq,enabled,t,*q=struct.unpack('>4Id6d',data)
            assert magic==0x52423243 and version==1
            received.append(q);time+=.002
            response=struct.pack('>4Id12d',0x52423246,1,seq,0,time,*([.25]*6),*([0.]*6))
            server.sendto(response,('127.0.0.1',5007))
    worker=threading.Thread(target=physics,daemon=True);worker.start()
    registry=subprocess.Popen([str(args.silkit/'bin/sil-kit-registry')],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
    bank=None
    try:
        import time
        time.sleep(.5)
        bank=subprocess.Popen([str(args.build/'rb2_physics_motor_bank')],stdout=subprocess.DEVNULL)
        result=subprocess.run([str(args.build/'can_client_test')],timeout=15)
        assert result.returncode==0 and received
        print('CAN physics bridge integration PASS; physical fixture is not Multibody validation.')
    finally:
        stop.set();worker.join(timeout=1);server.close()
        for process in (bank,registry):
            if process is not None:
                process.terminate()
                try:process.wait(timeout=3)
                except subprocess.TimeoutExpired:process.kill();process.wait()

if __name__=='__main__':main()
