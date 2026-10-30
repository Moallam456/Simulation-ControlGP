#include "CANComm/SILKit/silkit_can_backend.h"
#include "ServoDrive/AvatarM/avatar_m_pdo.h"
#include <chrono>
#include <thread>
#include <cstdio>
#include <cstdlib>
static void send(CanBackend &b,CanFrame &f){if(can_backend_send(&b,&f)!=CAN_BACKEND_OK)std::exit(2);}
int main(){
    CanBackend b{};SilKitCanBackendConfig c{"PhysicsTestController","TestCAN","CAN1","silkit://localhost:8500",1000000};
    if(!silkit_can_backend_create(&b,&c)||!silkit_can_backend_wait_ready(&b,5000))return 2;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    CanFrame f{};f.id=0;f.dlc=2;f.data[0]=1;send(b,f);
    for(int stage=0;stage<4;stage++)for(int i=1;i<=6;i++){
        f={};f.id=0x600+i;f.dlc=8;f.data[0]=stage==0?0x2f:0x2b;
        f.data[1]=stage==0?0x60:0x40;f.data[2]=0x60;f.data[4]=stage==0?7:stage==1?6:stage==2?7:15;send(b,f);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    for(int i=1;i<=6;i++){if(!avatar_m_build_rpdo4(i,123456,&f))return 2;send(b,f);}
    f={};f.id=0x80;f.dlc=0;send(b,f);
    auto start=std::chrono::steady_clock::now();unsigned mask=0;
    while(std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count()<3){
        auto r=can_backend_receive(&b,&f);
        if(r==CAN_BACKEND_OK && f.id>=0x481 && f.id<=0x486){
            AvatarMFeedback feedback{};
            if(!avatar_m_parse_tpdo4(f.id-0x480,&f,&feedback))return 3;
            int32_t actual=feedback.actual_position;
            // Python physical fixture reports q=0.25 rad on every axis.
            if(actual==123456||actual!=65190){std::fprintf(stderr,"Not physical feedback: %d\n",actual);return 4;}
            mask|=1U<<(f.id-0x481);
        }
        if(mask==63){std::puts("Six CAN nodes returned measured physics, not commanded positions.");can_backend_close(&b);return 0;}
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    can_backend_close(&b);std::fprintf(stderr,"Missing physical feedback: mask %u\n",mask);return 5;
}
