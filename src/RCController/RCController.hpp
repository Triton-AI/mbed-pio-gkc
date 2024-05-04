#ifndef RC_CONTROLLER_HPP_
#define RC_CONTROLLER_HPP_

#include "config.hpp"
#include "math.h"
#include "elrs_receiver.hpp"
#include "tai_gokart_packet/gkc_packets.hpp"
#include "Watchdog/watchable.hpp"
#include <Thread.h>

namespace tritonai::gkc
{

enum RatioMode {
    THROTTLE_RATIO = 0,
    EMPTY_RATIO = 1,
    BRAKE_RATIO = 2,
} ;

struct Translation
{
    double normalize(int analogValue);
    double steering(int steerVal);
    double throttle(int throttleVal);
    double throttle_ratio(int throttleVal);
    bool keep_constant_thr(int throttleVal);
    double brake(int brakeVal);
    bool is_active(int swith1, int swith2);
    AutonomyMode getAutonomyMode(int rightTriVal);
    RatioMode getRatioMode(int leftTriVal);
};

class RCController : public Watchable
{
    public:
    explicit RCController(GkcPacketSubscriber *sub);
    const RCControlGkcPacket& getPacket(){ 
        _is_ready = false;
        return _packet;
    }

    protected:
    void update();
    Translation Map;
    Thread _rc_thread{osPriorityNormal, OS_STACK_SIZE*2, nullptr, "rc_thread"};
    void watchdog_callback();
    

    private:
    elrc_receiver _receiver;
    RCControlGkcPacket _packet{};
    bool _is_ready;
    GkcPacketSubscriber *_sub;
    float current_throttle=0.0;
    float brake_ratio=0.5;
    float throttle_ratio=0.0;
    RatioMode ratio_mode=EMPTY_RATIO;
};

} // namespace tritonai::gkc

#endif  // RC_CONTROLLER_HPP_
