#ifndef RB2_PHYSICS_PROTOCOL_H
#define RB2_PHYSICS_PROTOCOL_H
#include <cstdint>
#include <cstring>
#include <cmath>
namespace rb2 {
inline void u32(unsigned char *p,uint32_t n){for(int i=3;i>=0;--i){p[i]=static_cast<unsigned char>(n);n>>=8;}}
inline uint32_t u32(const unsigned char *p){uint32_t n=0;for(int i=0;i<4;i++)n=(n<<8)|p[i];return n;}
inline void f64(unsigned char *p,double x){uint64_t n;std::memcpy(&n,&x,8);for(int i=7;i>=0;--i){p[i]=static_cast<unsigned char>(n);n>>=8;}}
inline double f64(const unsigned char *p){uint64_t n=0;for(int i=0;i<8;i++)n=(n<<8)|p[i];double x;std::memcpy(&x,&n,8);return x;}
inline void command(unsigned char b[72],uint32_t seq,bool enabled,double t,const double q[6]){
    u32(b,0x52423243);u32(b+4,1);u32(b+8,seq);u32(b+12,enabled?1:0);f64(b+16,t);
    for(int i=0;i<6;i++)f64(b+24+8*i,q[i]);
}
inline bool feedback(const unsigned char *b,int size,uint32_t &seq,uint32_t &fault,double &t,double q[6]){
    if(size!=120||u32(b)!=0x52423246||u32(b+4)!=1)return false;
    seq=u32(b+8);fault=u32(b+12);t=f64(b+16);if(!std::isfinite(t))return false;
    for(int i=0;i<6;i++){q[i]=f64(b+24+8*i);if(!std::isfinite(q[i]))return false;}return true;
}
}
#endif
