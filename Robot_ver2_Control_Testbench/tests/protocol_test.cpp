#include "../integration/physics_protocol.h"
#include <cassert>
#include <cstdio>
int main(){
    unsigned char b[120]{};double q[6]={0,.1,-.2,.3,-.4,.5},r[6],t;uint32_t seq,fault;
    rb2::command(b,42,true,1.25,q);assert(rb2::u32(b)==0x52423243);assert(rb2::u32(b+8)==42);
    assert(rb2::f64(b+16)==1.25);for(int i=0;i<6;i++)assert(rb2::f64(b+24+i*8)==q[i]);
    assert(!rb2::feedback(b,72,seq,fault,t,r));
    rb2::u32(b,0x52423246);rb2::u32(b+12,0);
    assert(rb2::feedback(b,120,seq,fault,t,r));assert(seq==42&&t==1.25&&!fault);
    rb2::f64(b+24,NAN);assert(!rb2::feedback(b,120,seq,fault,t,r));
    std::puts("Physics protocol encoding/length/finite-value tests passed.");
}
