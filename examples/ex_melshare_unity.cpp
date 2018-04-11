#include "MEII/Unity/UnityEmgRtc.hpp"
#include <MEL/Utility/Console.hpp>
#include <MEL/Core/Clock.hpp>
#include <MEL/Utility/System.hpp>
#include <MEL/Core/Timer.hpp>
#include <MEL/Math/Functions.hpp>
#include <vector>
#include <string>
#include <MEL/Utility/Windows/Keyboard.hpp>
#include <MEL/Logging/Log.hpp>

using namespace mel;
using namespace meii;

ctrl_bool stop(false);
bool handler(CtrlEvent event) {
    stop = true;
    return true;
}


int main() {

    // register ctrl-c handler
    register_ctrl_handler(handler);

    // initialize default MEL logger
    init_logger();

    // enable Windows realtime
    enable_realtime();

    // launch unity game
    UnityEmgRtc game;
    game.launch();

    // initialize timer
    Timer timer(milliseconds(1), Timer::Hybrid);

    // construct clock to regulate key presses
    Clock keypress_refract_clock;
    Time keypress_refract_time = seconds(1);

    // intialize experimental conditions
    int hand_num = 1; ///< 0 or 1 for Left or Right arm of the user, respectively
    int dof; ///< 0-3 is single-dof; 4-5 is multi-dof
    int num_classes; ///< number of classes based on the dof, 2 for single, 4 for multi
    int condition; ///< 0 = calibration; 1 = training; 2 = blind testing; 3 = full testing
    bool menu; ///< whether or not the main menu is currently selected

    // initialize melshare for showing effort on scope
    MelShare ms_effort("melscope_effort");

    // initialize local variables
    double prev_scene_num = -1.0;
    int current_class = -1;
    int prev_class = -1;   
    bool prev_menu;
    bool effort_large = false;
    std::vector<double> effort_ranges = { 0, 100, 50, 200 };
    double current_effort = effort_ranges[1];

    // initialize effort range
    game.set_effort_range(effort_ranges[0], effort_ranges[1]);
    
    timer.restart();
    while (!stop) {

        game.set_experiment_conditions(hand_num, dof, num_classes, condition, menu);
        
        if (menu != prev_menu && menu)
            game.set_target(-1);
        prev_menu = menu;
        
        if (!menu ) {
            current_class = Keyboard::is_any_num_key_pressed();
            if (current_class != prev_class) {
                if (current_class >= 0) {
                    game.set_target(current_class);
                }               
            }
            prev_class = current_class;

            // set the arrow magnitude
            if (game.get_target_label() > 0 && game.get_target_label() <= num_classes) {
                if (Keyboard::is_key_pressed(Key::Down)) {
                    current_effort -= 0.05;
                    game.set_effort(current_effort);
                }
                else if (Keyboard::is_key_pressed(Key::Up)) {
                    current_effort += 0.05;
                    game.set_effort(current_effort);
                }
                ms_effort.write_data({ current_effort });
            }
        }

        // toggle the effort range
        if (Keyboard::is_key_pressed(Key::Enter)) {
            if (keypress_refract_clock.get_elapsed_time() > keypress_refract_time) {
                if (effort_large) {
                    game.set_effort_range(effort_ranges[0], effort_ranges[1]);
                    effort_large = false;
                }
                else {
                    game.set_effort_range(effort_ranges[2], effort_ranges[3]);
                    effort_large = true;
                }
                game.set_effort(current_effort);
                keypress_refract_clock.restart();
            }
        }


        //scene_num = ms_scene.read_data();

        //if (scene_num.size() > 0) {
        //    if (scene_num[0] != prev_scene_num) {
        //        print("Scene number is " + std::to_string(scene_num[0]));
        //        set_experiment_conditions((int)scene_num[0], dof, condition);
        //        if (scene_num[0] == 0) {
        //            current_target[0] = -1.0;
        //            ms_target.write_data(current_target);
        //        }
        //    }
        //    prev_scene_num = scene_num[0];


        //    if (scene_num[0] > 0) {
        //        // set the target
        //        current_class = is_any_num_key_pressed();
        //        if (current_class != prev_class) {
        //            if (current_class >= 0) {
        //                print("User input is " + std::to_string(current_class));
        //                if (current_class == 0) {
        //                    current_target[0] = 0.0;
        //                }
        //                else {
        //                    current_target = set_viz_target_num(current_class, hand_num, dof);
        //                }
        //                print("Visual target sent is " + std::to_string(current_target[0]));
        //            }
        //            ms_target.write_data(current_target);
        //        }
        //        prev_class = current_class;

        //        // set the arrow magnitude
        //        if (Keyboard::is_key_pressed(Key::Down)) {
        //            current_effort[0] = saturate(current_effort[0] - 0.001, 0.0, 3.0);
        //            ms_effort.write_data(current_effort);
        //        }
        //        else if (Keyboard::is_key_pressed(Key::Up)) {
        //            current_effort[0] = saturate(current_effort[0] + 0.001, 0.0, 3.0);
        //            ms_effort.write_data(current_effort);
        //        }
        //    }
        //}             

        timer.wait();
        
    }

    disable_realtime();
    return 0;
}
