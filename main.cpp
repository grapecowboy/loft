#include "simulator.hpp"

using SS = SensorState;

static constexpr Clock::Time TIME_STEP = 10; ///< advance simulator by 10s for each step

/// At T+0, non-stop traffic in all directions. continues for 5 minutes
static const Scenario SCENARIO_1 = 
{   //            N-N        N-W        S-S        S-E        E-E        E-N        W-W        W-S
    { 0,  300,  { SS::SET,   SS::SET,   SS::SET,   SS::SET,   SS::SET,   SS::SET,   SS::SET,   SS::SET   }}
};

/// at T+0, there is  N-W and S-E traffic
/// at T+10, an infinite line of vehicles pulls up to the N-N sensor
/// at T+20, all N-W and S-E traffic stops
static const Scenario SCENARIO_2 = 
{   //             N-N        N-W        S-S        S-E        E-E        E-N        W-W        W-S
    { 0,   10,   { SS::CLEAR, SS::SET,   SS::CLEAR, SS::SET,   SS::CLEAR, SS::CLEAR, SS::CLEAR, SS::CLEAR }},
    { 10,  20,   { SS::SET,   SS::SET,   SS::CLEAR, SS::SET,   SS::CLEAR, SS::CLEAR, SS::CLEAR, SS::CLEAR }},
    { 20,  300,  { SS::SET,   SS::CLEAR, SS::CLEAR, SS::CLEAR, SS::CLEAR, SS::CLEAR, SS::CLEAR, SS::CLEAR }},
};

int main
(
    int argc, 
    char const *argv[]
)
{
    Simulator simulator(SCENARIO_1);
    auto &clock = simulator.clock();
    auto &sensors = simulator.sensors();

    std::cout << simulator.BANNER << std::endl;

    for(;;)
    {
        if (simulator.done())
        {
            break;
        }

        // controller.update(...);
        // simulator.update_signals(...);

        std::cout << simulator << std::endl;
        simulator.advance(TIME_STEP);
    }
}
