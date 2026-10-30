/* SIMULATION ONLY. Retains upstream AVATAR frame processing and SIL Kit backend.
 * TPDO4 is published only after a NEW physical measurement; never from targets. */
#include "avatar_m_silkit_node.h"
#include "../../ServoDrive/AvatarM/avatar_m_position.h"
#include "../../ServoDrive/AvatarM/avatar_m_pdo.h"
#include "../../ServoDrive/CiA402/cia402.h"
#include "physics_protocol.h"
#include <array>
#include <chrono>
#include <thread>
#include <cstdio>
#include <cstdlib>
#include <csignal>
#include <atomic>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
static std::atomic<bool> stop_requested{false};
static void stop(int){stop_requested=true;}
int main(){
    std::signal(SIGINT,stop);std::signal(SIGTERM,stop);
    const char *ip=std::getenv("ROBOT_PHYSICS_IP");if(!ip)ip="127.0.0.1";
    const char *uri=std::getenv("SILKIT_REGISTRY_URI");if(!uri)uri="silkit://localhost:8500";
    int sock=socket(AF_INET,SOCK_DGRAM,0);if(sock<0)return 1;
    sockaddr_in local{};local.sin_family=AF_INET;local.sin_port=htons(5007);local.sin_addr.s_addr=htonl(INADDR_ANY);
    if(bind(sock,reinterpret_cast<sockaddr*>(&local),sizeof local)<0){perror("physics feedback bind");close(sock);return 1;}
    sockaddr_in peer{};peer.sin_family=AF_INET;peer.sin_port=htons(5006);
    if(inet_pton(AF_INET,ip,&peer.sin_addr)!=1){close(sock);return 1;}
    std::array<AvatarMSilKitNode,6> nodes{};std::array<AvatarMPositionScale,6> scales{};
    const double home[6]={0,0,-2.0943951023931953,1.0471975511965976,2.2907446432425576,0};
    bool fatal=false,waiting=false;unsigned initialized=0;uint32_t seq=0,last_feedback=0;double last_time=-1;
    auto began=std::chrono::steady_clock::now(),last=began,request_time=began;
    for(int i=0;i<6;i++){
        char name[32],can[32];std::snprintf(name,sizeof name,"AvatarMotor%d",i+1);std::snprintf(can,sizeof can,"MotorCAN%d",i+1);
        AvatarMSilKitNodeConfig c{};c.node_id=i+1;c.participant_name=name;c.controller_name=can;c.network_name="CAN1";c.registry_uri=uri;c.bitrate=1000000;
        if(!avatar_m_silkit_node_create(&nodes[i],&c)){fatal=true;break;}initialized++;
        if(!silkit_can_backend_wait_ready(&nodes[i].backend,5000U)||!avatar_m_position_scale_default(&scales[i],50.0)){fatal=true;break;}
        int32_t raw;avatar_m_joint_rad_to_position_units(&scales[i],home[i],&raw);
        avatar_m_node_set_actual_position(&nodes[i].motor,raw);avatar_m_node_set_work_mode(&nodes[i].motor,1U);
    }
    while(!fatal&&!stop_requested){
        auto now=std::chrono::steady_clock::now();
        auto elapsed=std::chrono::duration_cast<std::chrono::milliseconds>(now-last).count();
        if(elapsed>0){for(auto &node:nodes)if(avatar_m_silkit_node_tick_ms(&node,static_cast<uint32_t>(elapsed))==AVATAR_M_SILKIT_ERROR)fatal=true;last=now;}
        unsigned sync_mask=0;
        for(int i=0;i<6&&!fatal;i++)for(unsigned budget=0;budget<128;budget++){
            CanFrame input{},response{};auto received=can_backend_receive(&nodes[i].backend,&input);
            if(received==CAN_BACKEND_WOULD_BLOCK)break;if(received!=CAN_BACKEND_OK){fatal=true;break;}
            auto result=avatar_m_node_process_frame(&nodes[i].motor,&input,&response);
            if(result==AVATAR_M_NODE_INVALID_FRAME){fatal=true;break;}
            if(input.id==0x80U){sync_mask|=1U<<i;continue;}
            // Upstream emits TPDO4 on RPDO4 using its last cached position.
            // Defer that response until MATLAB produces a new measurement.
            if(input.id==0x501U+static_cast<unsigned>(i))continue;
            if(result!=AVATAR_M_NODE_NO_RESPONSE&&can_backend_send(&nodes[i].backend,&response)!=CAN_BACKEND_OK){fatal=true;break;}
        }
        static unsigned pending_sync=0;pending_sync|=sync_mask;
        if(pending_sync==63U&&!waiting){
            pending_sync=0;double q[6];bool enabled=true;
            for(int i=0;i<6;i++){
                auto &m=nodes[i].motor;
                enabled=enabled&&((m.statusword&0x6fU)==0x27U)&&m.active_target_valid;
                int32_t raw=m.active_target_valid?m.active_target_position:m.actual_position;
                if(!avatar_m_position_units_to_joint_rad(&scales[i],raw,&q[i]))fatal=true;
            }
            // During commissioning hold the measured pose without faulting MATLAB.
            if(!enabled)for(int i=0;i<6;i++)avatar_m_position_units_to_joint_rad(&scales[i],nodes[i].motor.actual_position,&q[i]);
            unsigned char packet[72];rb2::command(packet,++seq,enabled,last_time<0?0:last_time,q);
            if(sendto(sock,packet,sizeof packet,0,reinterpret_cast<sockaddr*>(&peer),sizeof peer)!=sizeof packet){fatal=true;break;}
            waiting=true;request_time=now;
        }
        unsigned char packet[256];sockaddr_in sender{};socklen_t len=sizeof sender;
        int size=recvfrom(sock,packet,sizeof packet,MSG_DONTWAIT,reinterpret_cast<sockaddr*>(&sender),&len);
        if(size>0){
            uint32_t echoed=0,fault=0;double t=0,q[6];
            if(sender.sin_addr.s_addr!=peer.sin_addr.s_addr||!rb2::feedback(packet,size,echoed,fault,t,q)||fault){fatal=true;break;}
            if(echoed>seq||echoed<last_feedback||t<last_time){fatal=true;break;}
            if(t>last_time){
                last_time=t;last_feedback=echoed;
                for(int i=0;i<6;i++){
                    int32_t raw;if(!avatar_m_joint_rad_to_position_units(&scales[i],q[i],&raw)){fatal=true;break;}
                    avatar_m_node_set_actual_position(&nodes[i].motor,raw);
                    CanFrame response{};if(!avatar_m_build_tpdo4(i+1,raw,nodes[i].motor.statusword,&response)||
                        can_backend_send(&nodes[i].backend,&response)!=CAN_BACKEND_OK){fatal=true;break;}
                }
            }
            if(echoed==seq)waiting=false;
        }
        if(waiting&&std::chrono::duration<double>(now-request_time).count()>.15){std::fputs("MATLAB physical feedback timeout. No further TPDO updates.\n",stderr);fatal=true;}
        std::this_thread::sleep_for(std::chrono::microseconds(200));
    }
    for(unsigned i=0;i<initialized;i++)avatar_m_silkit_node_close(&nodes[i]);close(sock);
    if(fatal)std::fputs("Physics adapter fault: controller must inhibit on missing/faulted feedback. No automatic restart.\n",stderr);
    return fatal?1:0;
}
