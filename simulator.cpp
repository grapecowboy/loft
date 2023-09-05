#include "simulator.hpp"

#include <algorithm>
#include <iomanip>

const std::string Simulator::BANNER = 
"  Time     N-N   N-W   S-S   S-E   E-E   E-N   W-W   W-S\n"
"==========================================================";

static const std::string& signal_string
(
    SignalState signal
)
{
    static const std::vector<std::string> STRINGS = 
    {
        "RED",
        "YLW",
        "GRN"
    };

    return STRINGS[static_cast<unsigned>(signal)];
}

std::ostream& operator<<
(
    std::ostream& os, 
    const Simulator& simulator
)
{
    os << "[" << std::setw(4) << simulator.clock_.now() << "s] ";
    for (unsigned lane = 0; lane < simulator.signals_.size(); lane++)
    {
        auto signal = simulator.signals_[lane];
        os << " | " << signal_string(signal);
    }
    os << " | ";
    return os;
}



void Simulator::update_simulation
(
    void
)
{
    // find entry in scneario for current timestamp
    auto scenario_timeslice = 
        std::find_if(
            scenario_.begin(), scenario_.end(),
            [this](const SimulationTimeslice &state) 
            { 
                return 
                    ( clock_.now() >= state.start ) &&
                    ( clock_.now() < state.end );
            }
    );

    // advanced past end of scenario
    if (scenario_timeslice == scenario_.end())
    {
        done_ = true;
        return;
    }

    // update simulation state from scenario
    sensors_ = scenario_timeslice->sensors;
    currentRule_ = getNextRule();
    applyRule();
}

void Simulator::applyRule(void)
{
     for (unsigned lane = 0; lane < signals_.size(); lane++)
     {
            signals_[ lane ] = ruleLights_[currentRule_][lane];
     }

}

int Simulator::getRuleOpposingTraffic(void)
{
    std::vector<SensorState> trafficCount;


    for (auto i : opposition_[currentRule_])
    {
        if (sensors_[ i] == SensorState::SET)
            trafficCount.push_back(SensorState::SET);
    }

    return trafficCount.size();
}

int Simulator::getRuleTraffic(void)
{
    std::vector<SensorState> trafficCount;

    if (currentRule_==0)
    {
        if(sensors_[Lane::N_W]==SensorState::SET)
            trafficCount.push_back(SensorState::SET);
        
        if(sensors_[Lane::S_E]==SensorState::SET)
            trafficCount.push_back(SensorState::SET); 
    }
    else if (currentRule_==1)
    {
        if(sensors_[Lane::N_N]==SensorState::SET)
            trafficCount.push_back(SensorState::SET);
        
        if(sensors_[Lane::S_S]==SensorState::SET)
            trafficCount.push_back(SensorState::SET); 
    }
    else if (currentRule_==2)
    {
        if(sensors_[Lane::E_N]==SensorState::SET)
            trafficCount.push_back(SensorState::SET);
        
        if(sensors_[Lane::W_S]==SensorState::SET)
            trafficCount.push_back(SensorState::SET); 
    }
    else if (currentRule_==3)
    {
        if(sensors_[Lane::E_E]==SensorState::SET)
            trafficCount.push_back(SensorState::SET);
        
        if(sensors_[Lane::W_W]==SensorState::SET)
            trafficCount.push_back(SensorState::SET); 
    }

    return trafficCount.size();

}