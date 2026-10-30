#include "ServoDrive/AvatarM/avatar_m_position.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
int main(void){
    AvatarMPositionScale s;int32_t u;double measured;
    assert(avatar_m_position_scale_default(&s,50));s.direction=-1;s.zero_offset_units=12345;
    for(int i=-180;i<=180;i++){
        double q=i*3.141592653589793/180;
        assert(avatar_m_joint_rad_to_position_units(&s,q,&u));
        assert(avatar_m_position_units_to_joint_rad(&s,u,&measured));
        assert(fabs(q-measured)<=3.141592653589793/(50*32768)+1e-12);
    }
    puts("Encoder/reduction/direction/offset round trips passed (synthetic ratio).");return 0;
}
