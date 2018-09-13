#include <MEII/Unity/UnityEmgRtc.hpp>
#include <MEL/Core/Console.hpp>
#include <MEL/Core/Clock.hpp>
#include <MEL/Utility/System.hpp>
#include <MEL/Core/Timer.hpp>
#include <MEL/Math/Functions.hpp>
#include <vector>
#include <string>
#include <MEL/Devices/Windows/Keyboard.hpp>
#include <MEL/Logging/Log.hpp>
#include <MEII/EmgRealTimeControl/EmgRealTimeControl.hpp>

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

    // enable Windows realtime
    mel::enable_realtime();

    // launch unity game
    UnityEmgRtc game;
    game.launch();

    // initialize timer
    Timer timer(milliseconds(1), Timer::Hybrid);

    // construct clock to regulate key presses
    Clock keypress_refract_clock;
    Time keypress_refract_time = seconds(1);

	std::vector<std::string> arm_str = { "Left", "Right" };
	std::vector<std::string> dof_str = { "ElbowFE", "WristPS", "WristFE", "WristRU", "ElbowFE_and_WristPS", "WristFE_and_WristRU" };
	std::vector<std::string> phase_str = { "Calibration", "Training", "BlindTesting", "FullTesting" };
	

	// initialize local variables
	int current_class = -1;
	int prev_class = -1;
	bool effort_large = false;
	std::vector<double> effort_ranges = { 0, 100, 50, 200 };
	double current_effort = effort_ranges[1];
	bool current_center;

	// initialize effort range
	game.set_effort_range(effort_ranges[0], effort_ranges[1]);

	// prompt user for input to select which arm
	print("\r\nPress number key for selecting which arm is in the exo.");
	print("1 = Left arm");
	print("2 = Right arm");
	print("Press 'Escape' to exit the program.\r\n");
	int number_keypress;
	bool arm_selected = false;
	Arm arm = Left; // default
	while (!arm_selected && !stop) {

		// check for number keypress
		number_keypress = Keyboard::is_any_num_key_pressed();
		if (number_keypress >= 0) {
			if (keypress_refract_clock.get_elapsed_time() > keypress_refract_time) {
				if (number_keypress > 0 && number_keypress <= LastArm) {
					arm_selected = true;
					arm = (Arm)(number_keypress - 1);
					LOG(Info) << arm_str[arm] << " selected.";
				}
				keypress_refract_clock.restart();
			}
		}

		// check for exit key
		if (Keyboard::is_key_pressed(Key::Escape)) {
			stop = true;
			return 0;
		}

		// wait for remainder of sample period
		timer.wait();
	}

	// prompt user for input to select which DoF
	print("\r\nPress number key for selecting a single-DoF or multi-DoF trajectory.");
	print("1 = Elbow Flexion/Extension");
	print("2 = Wrist Pronation/Supination");
	print("3 = Wrist Flexion/Extension");
	print("4 = Wrist Radial/Ulnar Deviation");
	print("5 = Elbow Flexion/Extension and Wrist Pronation/Supination");
	print("6 = Wrist Flexion/Extension and Wrist Radial/Ulnar Deviation");
	print("Press 'Escape' to exit the program.\r\n");
	bool dof_selected = false;
	DoF dof = ElbowFE; // default
	std::size_t num_classes = 2; // number of classes based on the dof, 2 for single, 4 for multi
	while (!dof_selected && !stop) {

		// check for number keypress
		number_keypress = Keyboard::is_any_num_key_pressed();
		if (number_keypress >= 0) {
			if (keypress_refract_clock.get_elapsed_time() > keypress_refract_time) {
				if (number_keypress > 0 && number_keypress <= LastDoF) {
					dof = (DoF)(number_keypress - 1);
					dof_selected = true;
					LOG(Info) << dof_str[dof] << " selected.";
					if (is_single_dof(dof)) {
						num_classes = 2;						
					}
					else {
						num_classes = 4;
					}

				}
				keypress_refract_clock.restart();
			}
		}

		// check for exit key
		if (Keyboard::is_key_pressed(Key::Escape)) {
			stop = true;
			return 0;
		}

		// wait for remainder of sample period
		timer.wait();
	}


	// prompt user for input to select which phase
	print("\r\nPress number key to select experiment phase.");
	print("1 = Calibration of active/rest classifier.");
	print("2 = Training of directional classifer.");
	print("3 = Testing of directional classifier without robot motion.");
	print("4 = Testing of directional classifier with robot motion");
	print("Press 'Escape' to exit the program.\r\n");
	bool phase_selected = false;
	Phase phase = Calibration; // default
	while (!phase_selected && !stop) {

		// check for number keypress
		number_keypress = Keyboard::is_any_num_key_pressed();
		if (number_keypress >= 0) {
			if (keypress_refract_clock.get_elapsed_time() > keypress_refract_time) {
				if (number_keypress > 0 && number_keypress <= LastCondition) {
					phase = (Phase)(number_keypress - 1);
					LOG(Info) << phase_str[phase] << " selected.";
					phase_selected = true;
				}
				keypress_refract_clock.restart();
			}
		}

		// check for exit key
		if (Keyboard::is_key_pressed(Key::Escape)) {
			stop = true;
			return 0;
		}

		// wait for remainder of sample period
		timer.wait();
	}

	// set the experimental conditions for the unity game
	game.set_experiment_conditions(arm, dof, phase);
	

	// prompt user for input to control unity display
	print("\r\nPress number key to select target.");
	print("Press 'Escape' to exit the program.\r\n");
    while (!stop) {

		// update visualization target
		number_keypress = Keyboard::is_any_num_key_pressed();
		if (number_keypress >= 0) {
			if (keypress_refract_clock.get_elapsed_time() > keypress_refract_time) {
				if (number_keypress > 0) {
					game.set_target(number_keypress);
				}
				if (number_keypress == 0) {
					current_center = !current_center;
					game.set_center(current_center);
				}
				
				keypress_refract_clock.restart();
			}
		}

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
        }

        // toggle the effort range
        //if (Keyboard::is_key_pressed(Key::Enter)) {
        //    if (keypress_refract_clock.get_elapsed_time() > keypress_refract_time) {
        //        if (effort_large) {
        //            game.set_effort_range(effort_ranges[0], effort_ranges[1]);
        //            effort_large = false;
        //        }
        //        else {
        //            game.set_effort_range(effort_ranges[2], effort_ranges[3]);
        //            effort_large = true;
        //        }
        //        game.set_effort(current_effort);
        //        keypress_refract_clock.restart();
        //    }
        //}
   
		// check for exit key
		if (Keyboard::is_key_pressed(Key::Escape)) {
			stop = true;
		}		

        timer.wait();
        
    }

    mel::disable_realtime();
    return 0;
}
