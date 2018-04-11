#include <MEL/Communications/MelNet.hpp>
#include <MEL/Utility/Console.hpp>
#include <MEL/Core/Clock.hpp>
#include <MEL/Utility/System.hpp>
#include <MEL/Core/Timer.hpp>
#include <vector>
#include <string>

using namespace mel;

ctrl_bool stop(false);
bool handler(CtrlEvent event) {
    stop = true;
    return true;
}

int main() {

    // register ctrl-c handler
    register_ctrl_handler(handler);

    // enable Windows realtime
    enable_realtime();

    MelNet scene_num_share(50000, 50001, "10.98.64.109", true);

    Timer timer(milliseconds(1), Timer::Hybrid);

    std::vector<double> scene_num = { 0 };

    timer.restart();
    while (!stop) {

        if (scene_num_share.receive_message() == "scene_change") {
            scene_num = scene_num_share.receive_data();
            print(scene_num);
        }

        timer.wait();
        
    }

    disable_realtime();
    return 0;
}
