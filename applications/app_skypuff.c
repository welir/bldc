/*
	Copyright 2019 Kirill Kostiuchenko	kisel2626@gmail.com

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	The VESC firmware is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
/*
	This application turns VESC into paragliding winch controller.
	Some progress here: https://www.youtube.com/watch?v=KoNegc4SzxY

	To play with model winch, send terminal commands 'example_conf' 
	and 'alive 3000000' to set long communication timeout.

	Skypuff can't use VESC timeout mechanism because of smooth pull release.

	Terminal output format is unified to simplify parsing on the UI side.

	Each print consist of current state and comma delimeted messages.

	State: type message, type message, ...

	Types:
	  'pos': current position
	  'pull': pulling or breaking force
	  'speed': current speed
	  '--': warning text until end of printstatic void terminal_set_pull_force(int argc, const char **argv);

	Examples:
		UNITIALIZED: -- CONFIG OUT OF LIMITS -- Braking current 0A is out of limits (0.5, 20)
		BRAKING: pos 34.3m (3432 steps), pull 2.3Kg (6.2A)
*/

#include "app.h"
#include "ch.h"
#include "hal.h"

#include "comm_can.h"
#include "commands.h"
#include "encoder.h"
#include "hw.h"
#include "mc_interface.h"
#include "terminal.h"
#include "timeout.h"
#include "utils.h"

#include <math.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

// Uncomment to debug Smooth Motor Control from terminal
//#define DEBUG_SMOOTH_MOTOR

// Must be increased on skypuff_config struct update
// Don't forget to update HW limits structs
static const int config_version = 1;

// Delays between pereodically prints of state, position and forces
static const int short_print_delay = 500; // 0.5s, measured in control loop counts
static const int long_print_delay = 3000; // 3s
// Between smooth motor adjustments
static const int smooth_max_step_delay = 100; // 0.1s

// Terminal warnings about bad settings
const char *limits_w1 = "-- CONFIG OUT OF LIMITS --";
const char *limits_w2 = "-- Waiting for 'example_conf' or COMM_CUSTOM_APP_DATA";

// Threads
static THD_FUNCTION(my_thread, arg);
static THD_WORKING_AREA(my_thread_wa, 2048);

// Private functions
static void terminal_set_zero(int argc, const char **argv);
static void terminal_print_conf(int argc, const char **argv);
static void terminal_set_example_conf(int argc, const char **argv);
static void terminal_alive(int argc, const char **argv);
static void terminal_set_state(int argc, const char **argv);
static void terminal_set_pull_force(int argc, const char **argv);
#ifdef DEBUG_SMOOTH_MOTOR
static void terminal_smooth(int argc, const char **argv);
#endif

//static void terminal_set_conf(int argc, const char **argv);
//static void terminal_get_conf(int argc, const char **argv);

// Private variables
static volatile bool stop_now = true;
static volatile bool is_running = false;
static const volatile mc_configuration *mc_conf;
static int prev_abs_tac = 0;
static float prev_erpm = 0;
static int i;							   // Global control loop counter
static int prev_print = INT_MIN / 2;	   // Loop counter value of the last state print
static int prev_printed_tac = INT_MIN / 2; // Do not print the same position
static int alive_until = 0;				   // In good communication we trust until (i < alive_until)
static int pre_pull_start_time;			   // Save loop counter here when pre pull begins
static int takeoff_pull_start_time;		   // Save loop counter here when takeoff pull begins
static float terminal_pull_kg;			   // Pull force to set from terminal thread
static volatile int alive_inc = 0;		   // Communication timeout increment from terminal thread

// Winch state machine
typedef enum
{
	UNINITIALIZED,			   // Released motor, until some valid configuration set
	BRAKING,				   // Braking zone near take off
	SLOWING,				   // Next to braking zone for slowing down motor until config.slow_erpm
	SLOW,					   // Constant speed to reach zero
	UNWINDING,				   // Low force rope tension during step up or unwinder mode
	REWINDING,				   // Higher force fast rope winding to take off for unwinder mode
	PRE_PULL,				   // Pull while pilot stays on the ground
	TAKEOFF_PULL,			   // Pull to take off
	PULL,					   // Nominal pull
	FAST_PULL,				   // Pull a bit above nominal pull
	MANUAL_BRAKING,			   // Braking caused by operator or communication timeout
	MANUAL_SLOW_SPEED_UP,	  // Speed up until manual constant speed to reach zero direction
	MANUAL_SLOW,			   // Constant speed to reach zero from any position
	MANUAL_SLOW_BACK_SPEED_UP, // Speed up back until manual constant speed from zero direction
	MANUAL_SLOW_BACK,		   // Constant speed from zero
#ifdef DEBUG_SMOOTH_MOTOR
	MANUAL_DEBUG_SMOOTH, // Debug smooth motor movements with 'smooth' terminal commands
#endif
} skypuff_state;

static skypuff_state state;

// Commands from terminal thread
typedef enum
{
	DO_NOTHING,
	SET_ZERO,
	SET_MANUAL_BRAKING,
	SET_UNWINDING,
	SET_MANUAL_SLOW,
	SET_MANUAL_SLOW_BACK,
	SET_PULL_FORCE,
	SET_PRE_PULL,
	SET_TAKEOFF_PULL,
	SET_PULL,
	SET_FAST_PULL,
#ifdef DEBUG_SMOOTH_MOTOR
	SET_SMOOTH,
#endif
	GET_CONF,
	SET_CONF,
} skypuff_terminal_command;

static volatile skypuff_terminal_command terminal_command = DO_NOTHING;

/*
	Smooth Motor Control

	Respect pilot and do not pull or release him too sharply.
	Unwinding current is minimal step of applying pull or brake.

	Use config.amps_per_sec as force changing speed and
	smooth_max_step_delay as maximum step delay.

	In case of different current and target modes, 
	switch instantly if force is less then unwinding 
	and smoothly decrease until unwinding if above.

	Always brake instantly if braking zone.
*/
typedef enum
{
	MOTOR_RELEASED,
	MOTOR_CURRENT,
	MOTOR_BRAKING,
	MOTOR_SPEED,
} smooth_motor_mode;

typedef struct
{
	smooth_motor_mode mode;
	union {
		float current;
		float erpm;
	} param;
} smooth_motor_state;

static smooth_motor_state current_motor_state;
static smooth_motor_state target_motor_state;
#ifdef DEBUG_SMOOTH_MOTOR
static smooth_motor_state terminal_motor_state;
#endif
static int next_smooth_motor_adjustment; // Loop count for next motor adjustment
static int prev_smooth_motor_adjustment; // Loop count of previous motor adjustment

typedef struct
{
	float kg_to_amps;					// Winch force coefficient
	float amps_per_sec;					// Force change speed
	int rope_length;					// Winch rope length (used by interface only)
	int braking_length;					// Tachometer range for braking zone
	int passive_braking_length;			// Increase braking_length on passive_braking_length from BRAKING
	int overwinding;					// Tachometer range to overwind braking zone
	int slowing_length;					// Range after braking zone to release motor and slow down until slow_erpm
	float slow_erpm;					// Constant erpm for slow mode
	int rewinding_trigger_length;		// Fast rewinding after going back range
	int unwinding_trigger_length;		// Back to unwinding from rewinding if this range unwinded
	float pull_current;					// Winch nominal pulling force
	float pre_pull_k;					// pre_pull_k * pull_current = pull current when pilots stays ready
	float takeoff_pull_k;				// takeoff_pull_k * pull_current = pull current for take off
	float fast_pull_k;					// fast_pull_k * pull_current = fast pull current for high altitude
	int takeoff_trigger_length;			// Minimal movement for switching to start pull
	int pre_pull_timeout;				// Timeout before saving position after pre pull
	int takeoff_period;					// Time of start pulling force
	float brake_current;				// Braking force
	float manual_brake_current;			// Manual braking force, could be high to charge battery driving away
	float unwinding_current;			// Unwinding force
	float rewinding_current;			// Rewinding force
	float slow_max_current;				// On exceed max current go braking or unwinding
	float manual_slow_max_current;		// On exceed max current go manual braking
	float manual_slow_speed_up_current; // Speed up current until max_slow_erpm
	float manual_slow_erpm;				// Constant speed for manual rotation
} skypuff_config;

static skypuff_config config;
static skypuff_config set_config; // to update from terminal thread;

/*
	Skypuff relies on system drive settings to calculate rope meters
	This struct is necessary for limits only,
	actual values stored in mc_configuration struc.
*/
typedef struct
{
	int motor_poles;
	float wheel_diameter;
	float gear_ratio;
} skypuff_drive_limits;

static const skypuff_drive_limits min_drive_limits = {
	.motor_poles = 2,
	.wheel_diameter = 0.01, // In meters
	.gear_ratio = 0.05,
};

static const skypuff_drive_limits max_drive_limits = {
	.motor_poles = 60,
	.wheel_diameter = 2,
	.gear_ratio = 20, // Motor turns for 1 wheel turn
};

// Do we actually need meters and meters per second for limits?
static const skypuff_config min_config = {
	.kg_to_amps = 0.5,
	.amps_per_sec = 0.5,
	.rope_length = 5,
	.braking_length = 5,
	.passive_braking_length = 0,
	.overwinding = 3,
	.slowing_length = 3,
	.slow_erpm = 100,
	.rewinding_trigger_length = 10,
	.unwinding_trigger_length = 3,
	.pull_current = 1,
	.pre_pull_k = 0.1,
	.takeoff_pull_k = 0.3,
	.fast_pull_k = 1.05,
	.takeoff_trigger_length = 3,
	.pre_pull_timeout = 100, // 0.1 secs
	.takeoff_period = 100,
	.brake_current = 1,
	.manual_brake_current = 1,
	.unwinding_current = 1,
	.rewinding_current = 1,
	.slow_max_current = 1,
	.manual_slow_max_current = 1,
	.manual_slow_speed_up_current = 1,
	.manual_slow_erpm = 100,
};

static const skypuff_config max_config = {
	.kg_to_amps = 30,
	.amps_per_sec = 500,
	.rope_length = 5000 * 120, // 120 - maximum motor poles * 3
	.braking_length = 100 * 120,
	.passive_braking_length = 5000 * 120,
	.overwinding = 0, // must be half of received braking length
	.slowing_length = 100 * 120,
	.slow_erpm = 5000,
	.rewinding_trigger_length = 50 * 120,
	.unwinding_trigger_length = 10 * 120,
	.pull_current = 600, // Believe in gliders
	.pre_pull_k = 0.5,
	.takeoff_pull_k = 0.8,
	.fast_pull_k = 1.5,
	.takeoff_trigger_length = 5000 * 120,
	.pre_pull_timeout = 5000, // 5 secs
	.takeoff_period = 60000,  // 1 min
	.brake_current = 500,
	.manual_brake_current = 500,
	.unwinding_current = 50,
	.rewinding_current = 100,
	.slow_max_current = 50,
	.manual_slow_max_current = 50,
	.manual_slow_speed_up_current = 50,
	.manual_slow_erpm = 8000,
};

inline static const char *
state_str(const skypuff_state s)
{
	switch (s)
	{
	case UNINITIALIZED:
		return "UNITIALIZED";
	case BRAKING:
		return "BRAKING";
	case MANUAL_BRAKING:
		return "MANUAL_BRAKING";
	case MANUAL_SLOW_SPEED_UP:
		return "MANUAL_SLOW_SPEED_UP";
	case MANUAL_SLOW:
		return "MANUAL_SLOW";
	case MANUAL_SLOW_BACK_SPEED_UP:
		return "MANUAL_SLOW_BACK_SPEED_UP";
	case MANUAL_SLOW_BACK:
		return "MANUAL_SLOW_BACK";
	case UNWINDING:
		return "UNWINDING";
	case REWINDING:
		return "REWINDING";
	case SLOWING:
		return "SLOWING";
	case SLOW:
		return "SLOW";
	case PRE_PULL:
		return "PRE_PULL";
	case TAKEOFF_PULL:
		return "TAKEOFF_PULL";
	case PULL:
		return "PULL";
	case FAST_PULL:
		return "FAST_PULL";
#ifdef DEBUG_SMOOTH_MOTOR
	case MANUAL_DEBUG_SMOOTH:
		return "MANUAL_DEBUG_SMOOTH";
#endif
	default:
		return "UNKNOWN";
	}
}

// Units converting calculations
inline static float meters_per_rev(void)
{
	return mc_conf->si_wheel_diameter / mc_conf->si_gear_ratio * M_PI;
}

inline static float steps_per_rev(void)
{
	return mc_conf->si_motor_poles * 3;
}

inline static int meters_to_tac_steps(float meters)
{
	return round(meters / meters_per_rev() * steps_per_rev());
}

inline static float tac_steps_to_meters(int steps)
{
	return (float)steps / steps_per_rev() * meters_per_rev();
}

inline static float ms_to_erpm(float ms)
{
	float rps = ms / meters_per_rev();
	float rpm = rps * 60;

	return rpm * (mc_conf->si_motor_poles / 2);
}

inline static float erpm_to_ms(float erpm)
{
	float erps = erpm / 60;
	float rps = erps / (mc_conf->si_motor_poles / 2);

	return rps * meters_per_rev();
}

inline static const char *motor_mode_str(smooth_motor_mode m)
{
	switch (m)
	{
	case MOTOR_RELEASED:
		return "MOTOR_RELEASED";
	case MOTOR_BRAKING:
		return "MOTOR_BRAKING";
	case MOTOR_CURRENT:
		return "MOTOR_CURRENT";
	case MOTOR_SPEED:
		return "MOTOR_SPEED";
	default:
		return "MOTOR_UNKNOWN";
	}
}

#ifdef DEBUG_SMOOTH_MOTOR
inline static void snprintf_motor_state(const smooth_motor_state *s, char *buf, int buf_len)
{
	switch (s->mode)
	{
	case MOTOR_CURRENT:
	case MOTOR_BRAKING:
		// Print precisely to simplify debuggging
		snprintf(buf, buf_len, "%s (%.3fkg / %.2fA)", motor_mode_str(s->mode),
				 (double)(s->param.current / config.kg_to_amps), (double)s->param.current);
		break;
	case MOTOR_SPEED:
		snprintf(buf, buf_len, "%s (%.1fms / %.0f ERPM)", motor_mode_str(s->mode),
				 (double)erpm_to_ms(s->param.erpm), (double)s->param.erpm);
		break;
	default:
		snprintf(buf, buf_len, "%s", motor_mode_str(s->mode));
		break;
	}
}
#endif

inline static void smooth_motor_instant_brake(void)
{
#ifdef DEBUG_SMOOTH_MOTOR
	const int buf_len = 100;
	char buf[buf_len];
	snprintf_motor_state(&target_motor_state, buf, buf_len);
	commands_printf("%s: loop %d, -- set brake instantly %s", state_str(state), i, buf);
#endif
	current_motor_state = target_motor_state;
	mc_interface_set_brake_current(target_motor_state.param.current);

	next_smooth_motor_adjustment = INT_MAX;
	prev_smooth_motor_adjustment = i;
	return;
}

inline static void smooth_motor_instant_current(void)
{
#ifdef DEBUG_SMOOTH_MOTOR
	const int buf_len = 100;
	char buf[buf_len];
	snprintf_motor_state(&target_motor_state, buf, buf_len);
	commands_printf("%s: loop %d, -- set current instantly %s", state_str(state), i, buf);
#endif

	current_motor_state = target_motor_state;

	mc_interface_set_current(target_motor_state.param.current);

	next_smooth_motor_adjustment = INT_MAX;
	prev_smooth_motor_adjustment = i;
	return;
}

inline static int smooth_motor_prev_adjustment_delay(void)
{
	int prev_adjustment_delay = i - prev_smooth_motor_adjustment;

	if (prev_adjustment_delay > smooth_max_step_delay)
		prev_adjustment_delay = smooth_max_step_delay;

	return prev_adjustment_delay;
}

// Do not complicate this calculation with jumps over unwinding boundaries
inline static int smooth_calc_next_delay(const float c1, const float c2)
{
	double secs_delay = fabs(c2 - c1) / (double)config.amps_per_sec;
	int millis_delay = (int)(secs_delay * (double)1000.0);

#ifdef DEBUG_SMOOTH_MOTOR
	commands_printf("%s: loop %d, smooth_calc_next_delay(%.5fA, %.5fA) secs_delay: %.5fs, millis: %d",
					state_str(state), i, (double)c1, (double)c2, (double)secs_delay, millis_delay);
#endif

	// Cut maximum delay with smooth_max_step_delay
	millis_delay = smooth_max_step_delay < millis_delay ? smooth_max_step_delay : millis_delay;

	return millis_delay;
}

inline static bool smooth_is_between_unwinding(const float c)
{
	return c > -config.unwinding_current && c < config.unwinding_current;
}

inline static float smooth_signed_unwinding_by(float c)
{
	return c >= 0 ? config.unwinding_current : -config.unwinding_current;
}

// Returns new current and next adjustment delay
inline static float smooth_calc_step(const float c1,
									 const float c2,
									 const int prev_adjustment_delay, // delay from previous adjustment
									 int *next_delay)
{
	// Respect force changing speed and delay from previous force adjustment
	float step = config.amps_per_sec * (float)prev_adjustment_delay / (float)1000.0;

	float c;

	// Target below current?
	if (c2 < c1)
	{
		c = c1 - step;
		if (c <= c2)
			c = c2;
	}
	else
	{
		c = c1 + step;
		if (c >= c2)
			c = c2;
	}

	// Respect unwinding boundaries
	if (smooth_is_between_unwinding(c))
	{
		if (smooth_is_between_unwinding(c2))
			return c2;
		else if (c2 < 0)
			c = -config.unwinding_current;
		else
			c = config.unwinding_current;
	}
	else
	{
	}

	*next_delay = smooth_calc_next_delay(c, c2);

	// Prevent extra adjustments
	if (*next_delay <= 1)
		*next_delay = 0;

	return c;
}

inline static void smooth_motor_adjustment(const int cur_tac, const int abs_tac)
{
	int prev_adjustment_delay = smooth_motor_prev_adjustment_delay();

#ifdef DEBUG_SMOOTH_MOTOR
	const int buf_len = 100;
	char current_buf[buf_len], target_buf[buf_len];
	snprintf_motor_state(&current_motor_state, current_buf, buf_len);
	snprintf_motor_state(&target_motor_state, target_buf, buf_len);
	commands_printf("%s: loop %d, -- smooth_motor_adjustment(cur_tac %d), prev_delay %d, %s -> %s",
					state_str(state), i, cur_tac, prev_adjustment_delay,
					current_buf, target_buf);
#endif

	float step_current;
	float signed_unwinding_current;
	int next_delay = 0; // Instant set by default

	switch (target_motor_state.mode)
	{
	case MOTOR_BRAKING:
		// Braking zone?
		if (abs_tac <= config.braking_length)
		{
#ifdef DEBUG_SMOOTH_MOTOR
			commands_printf("%s: loop %d, -- braking zone detected", state_str(state), i);
#endif
			smooth_motor_instant_brake();
			return;
		}

		switch (current_motor_state.mode)
		{
		case MOTOR_BRAKING:
			step_current = smooth_calc_step(current_motor_state.param.current,
											target_motor_state.param.current,
											prev_adjustment_delay,
											&next_delay);

			// Finished?
			if (!next_delay)
			{
				smooth_motor_instant_brake();
				return;
			}

			mc_interface_set_brake_current(step_current);
			current_motor_state.param.current = step_current;

			prev_smooth_motor_adjustment = i;
			next_smooth_motor_adjustment = i + next_delay;
			break;
		case MOTOR_CURRENT:
			// Smoothly decrease current until unwinding current and start braking
			signed_unwinding_current = smooth_signed_unwinding_by(current_motor_state.param.current);

			step_current = smooth_calc_step(current_motor_state.param.current,
											signed_unwinding_current,
											prev_adjustment_delay,
											&next_delay);

			// signed_unwinding_current could be reached on this adjustment?
			if (!next_delay)
			{
				current_motor_state.mode = MOTOR_BRAKING;
				current_motor_state.param.current = 0;

				smooth_motor_adjustment(cur_tac, abs_tac);
				return;
			}

			mc_interface_set_current(step_current);
			current_motor_state.param.current = step_current;

			prev_smooth_motor_adjustment = i;
			next_smooth_motor_adjustment = i + next_delay;

			break;
		default:
			// Process MOTOR_RELEASED and MOTOR_SPEED here
			// Just smoothly increase braking
			current_motor_state.mode = MOTOR_BRAKING;
			current_motor_state.param.current = 0;

			smooth_motor_adjustment(cur_tac, abs_tac);
			break;
		}

		break;
	case MOTOR_CURRENT: // Target state
		switch (current_motor_state.mode)
		{
		case MOTOR_CURRENT:
			step_current = smooth_calc_step(current_motor_state.param.current,
											target_motor_state.param.current,
											prev_adjustment_delay,
											&next_delay);

			// Finished?
			if (!next_delay)
			{
				smooth_motor_instant_current();
				return;
			}

			mc_interface_set_current(step_current);
			current_motor_state.param.current = step_current;

			prev_smooth_motor_adjustment = i;
			next_smooth_motor_adjustment = i + next_delay;
			break;
		case MOTOR_BRAKING:
			// Smoothly decrease braking until unwinding current and start current
			step_current = smooth_calc_step(current_motor_state.param.current,
											config.unwinding_current,
											prev_adjustment_delay,
											&next_delay);

			// signed_unwinding_current could be reached on this adjustment?
			if (!next_delay)
			{
				current_motor_state.mode = MOTOR_CURRENT;
				current_motor_state.param.current = 0;

				smooth_motor_adjustment(cur_tac, abs_tac);
				return;
			}

			mc_interface_set_brake_current(step_current);
			current_motor_state.param.current = step_current;

			prev_smooth_motor_adjustment = i;
			next_smooth_motor_adjustment = i + next_delay;

			break;
		default:
			// Process MOTOR_RELEASED and MOTOR_SPEED here
			// Just smoothly increase current
			current_motor_state.mode = MOTOR_CURRENT;
			current_motor_state.param.current = 0;

			smooth_motor_adjustment(cur_tac, abs_tac);
			break;
		}
		break;

	default:
		commands_printf("%s: -- Wrong smooth motor adjustment target mode: '%s'",
						state_str(state), motor_mode_str(target_motor_state.mode));
		next_smooth_motor_adjustment = INT_MAX;
		return;
	}
}

inline static void smooth_motor_release(void)
{
#ifdef DEBUG_SMOOTH_MOTOR
	commands_printf("%s: loop %d, -- set instant MOTOR_RELEASED, smooth_motor_release()", state_str(state), i);
#endif

	target_motor_state.mode = MOTOR_RELEASED;
	target_motor_state.param.current = 0;

	mc_interface_release_motor();

	current_motor_state = target_motor_state;
	next_smooth_motor_adjustment = INT_MAX;
	prev_smooth_motor_adjustment = i;
}

inline static void smooth_motor_speed(const float erpm)
{
#ifdef DEBUG_SMOOTH_MOTOR
	commands_printf("%s: loop %d, -- set instant MOTOR_SPEED smooth_motor_speed(%.0f ERPM)",
					state_str(state), i, (double)erpm);
#endif
	target_motor_state.mode = MOTOR_SPEED;
	target_motor_state.param.erpm = erpm;

	mc_interface_set_pid_speed(erpm);

	current_motor_state = target_motor_state;
	next_smooth_motor_adjustment = INT_MAX;
	prev_smooth_motor_adjustment = i;
}

inline static void smooth_motor_brake(const int cur_tac, const int abs_tac, const float current)
{
#ifdef DEBUG_SMOOTH_MOTOR
	commands_printf("%s: loop %d, -- smooth_motor_brake(cur_tac %d, %.2fA)",
					state_str(state), i, cur_tac, (double)current);
#endif

	target_motor_state.mode = MOTOR_BRAKING;
	target_motor_state.param.current = current;

	smooth_motor_adjustment(cur_tac, abs_tac);
}

inline static void smooth_motor_current(const int cur_tac, const int abs_tac, const float current)
{
#ifdef DEBUG_SMOOTH_MOTOR
	commands_printf("%s: loop %d, -- smooth_motor_current(%.2fA)",
					state_str(state), i, (double)current);
#endif

	target_motor_state.mode = MOTOR_CURRENT;
	target_motor_state.param.current = current;

	smooth_motor_adjustment(cur_tac, abs_tac);
}

inline static bool is_int_out_of_limits(const char *name, const char *units,
										const int val, const int min, const int max)
{
	if (val >= min && val <= max)
		return false;

	commands_printf("%s: %s %s: %d %s is out of limits [%d, %d] %s",
					state_str(state), limits_w1, name, val, units, min, max, limits_w2);
	return true;
}

inline static bool is_float_out_of_limits(const char *name, const char *units,
										  const float val, const float min, const float max)
{
	if (val >= min && val <= max)
		return false;

	commands_printf("%s: %s %s: %.5f %s is out of limits [%.5f, %.5f] %s",
					state_str(state), limits_w1, name, (double)val, units, (double)min, (double)max, limits_w2);
	return true;
}

inline static bool is_pull_out_of_limits(const char *name,
										 const float amps, const float min, const float max)
{
	if (amps >= min && amps <= max)
		return false;

	commands_printf("%s: %s %s: %.1fA (%.2f Kg) is out of limits [%.1fA (%.2f Kg), %.1fA (%.2f Kg)] %s",
					state_str(state), limits_w1, name,
					(double)amps, (double)(amps / config.kg_to_amps),
					(double)min, (double)(min / config.kg_to_amps),
					(double)max, (double)(max / config.kg_to_amps),
					limits_w2);
	return true;
}

inline static bool is_distance_out_of_limits(const char *name,
											 const int steps, const int min, const int max)
{
	if (steps >= min && steps <= max)
		return false;

	commands_printf("%s: %s %s: %d steps (%.2f meters) is out of limits [%d (%.2fm), %d (%.2fm)] %s",
					state_str(state), limits_w1, name,
					steps, (double)tac_steps_to_meters(steps),
					min, (double)tac_steps_to_meters(min),
					max, (double)tac_steps_to_meters(max),
					limits_w2);
	return true;
}

inline static bool is_speed_out_of_limits(const char *name,
										  const float erpm, const float min, const float max)
{
	if (erpm >= min && erpm <= max)
		return false;

	commands_printf("%s: %s %s: %.1f ERPM (%.1f m/s) is out of limits [%.1f (%.1f m/s), %.1f (%.1f m/s)] %s",
					state_str(state), limits_w1, name,
					(double)erpm, (double)erpm_to_ms(erpm),
					(double)min, (double)erpm_to_ms(min),
					(double)max, (double)erpm_to_ms(max),
					limits_w2);
	return true;
}

static bool is_config_out_of_limits(const skypuff_config *conf)
{
	// Check mc_conf also
	return is_int_out_of_limits("motor_poles", "p", mc_conf->si_motor_poles,
								min_drive_limits.motor_poles, max_drive_limits.motor_poles) ||
		   is_float_out_of_limits("wheel_diameter", "mm", mc_conf->si_wheel_diameter * (float)1000,
								  min_drive_limits.wheel_diameter * (float)1000.0,
								  max_drive_limits.wheel_diameter * (float)1000.0) ||
		   is_float_out_of_limits("gear_ratio", "turn(s)", mc_conf->si_gear_ratio,
								  min_drive_limits.gear_ratio, max_drive_limits.gear_ratio) ||
		   // finally our conf
		   is_float_out_of_limits("kg_to_amps", "KgA", conf->kg_to_amps,
								  min_config.kg_to_amps, max_config.kg_to_amps) ||
		   is_float_out_of_limits("amps_per_sec", "A/Sec", conf->amps_per_sec,
								  min_config.amps_per_sec, max_config.amps_per_sec) ||
		   is_distance_out_of_limits("rope_length", conf->rope_length,
									 min_config.rope_length, max_config.rope_length) ||
		   is_distance_out_of_limits("braking_length", conf->braking_length,
									 min_config.braking_length, max_config.braking_length) ||
		   is_distance_out_of_limits("passive_braking_length", conf->passive_braking_length,
									 min_config.passive_braking_length, max_config.passive_braking_length) ||
		   is_distance_out_of_limits("overwinding", conf->overwinding,
									 min_config.overwinding, conf->braking_length / 2) ||
		   is_distance_out_of_limits("slowing_length", conf->slowing_length,
									 min_config.slowing_length, max_config.slowing_length) ||
		   is_distance_out_of_limits("rewinding_trigger_length", conf->rewinding_trigger_length,
									 min_config.rewinding_trigger_length, max_config.rewinding_trigger_length) ||
		   is_distance_out_of_limits("unwinding_trigger_length", conf->unwinding_trigger_length,
									 min_config.unwinding_trigger_length, max_config.unwinding_trigger_length) ||
		   is_distance_out_of_limits("takeoff_trigger_length", conf->takeoff_trigger_length,
									 min_config.takeoff_trigger_length, max_config.takeoff_trigger_length) ||
		   is_speed_out_of_limits("slow_erpm", conf->slow_erpm,
								  min_config.slow_erpm, max_config.slow_erpm) ||
		   is_speed_out_of_limits("manual_slow_erpm", conf->manual_slow_erpm,
								  min_config.manual_slow_erpm, max_config.manual_slow_erpm) ||
		   is_pull_out_of_limits("pull_current", conf->pull_current,
								 min_config.pull_current, max_config.pull_current) ||
		   is_pull_out_of_limits("brake_current", conf->brake_current,
								 min_config.brake_current, max_config.brake_current) ||
		   is_pull_out_of_limits("manual_brake_current", conf->manual_brake_current,
								 min_config.manual_brake_current, max_config.manual_brake_current) ||
		   is_pull_out_of_limits("unwinding_current", conf->unwinding_current,
								 min_config.unwinding_current, max_config.unwinding_current) ||
		   is_pull_out_of_limits("rewinding_current", conf->rewinding_current,
								 conf->unwinding_current, max_config.rewinding_current) ||
		   is_pull_out_of_limits("slow_max_current", conf->slow_max_current,
								 min_config.slow_max_current, max_config.slow_max_current) ||
		   is_pull_out_of_limits("manual_slow_max_current", conf->manual_slow_max_current,
								 min_config.manual_slow_max_current, max_config.manual_slow_max_current) ||
		   is_pull_out_of_limits("manual_slow_speed_up_current", conf->manual_slow_speed_up_current,
								 min_config.manual_slow_speed_up_current, max_config.manual_slow_speed_up_current) ||
		   is_float_out_of_limits("pre_pull_k", "%%", conf->pre_pull_k * (float)100,
								  min_config.pre_pull_k * (float)100, max_config.pre_pull_k * (float)100) ||
		   is_float_out_of_limits("takeoff_pull_k", "%%", conf->takeoff_pull_k * (float)100,
								  min_config.takeoff_pull_k * (float)100, max_config.takeoff_pull_k * (float)100) ||
		   is_float_out_of_limits("fast_pull_k", "%%", conf->fast_pull_k * (float)100,
								  min_config.fast_pull_k * (float)100, max_config.fast_pull_k * (float)100) ||
		   is_int_out_of_limits("pre_pull_timeout", "millis", conf->pre_pull_timeout,
								min_config.pre_pull_timeout, max_config.pre_pull_timeout) ||
		   is_int_out_of_limits("takeoff_period", "millis", conf->takeoff_period,
								min_config.takeoff_period, max_config.takeoff_period);
}

// State setters
inline static void brake_state(const int cur_tac, const skypuff_state new_state, const float current,
							   const char *additional_msg)
{
	// We can call breking inside braking states to renew control timeout
	if (state != new_state)
	{
		prev_print = i;
		prev_printed_tac = cur_tac;
		commands_printf("%s: pos %.2fm (%d steps), breaking %.1fKg (%.1fA)%s",
						state_str(new_state),
						(double)tac_steps_to_meters(cur_tac), cur_tac,
						(double)(current / config.kg_to_amps), (double)current,
						additional_msg);
	}

	state = new_state;

	prev_abs_tac = abs(cur_tac);
	smooth_motor_brake(cur_tac, prev_abs_tac, current);
	timeout_reset();
}

inline static void brake(const int cur_tac)
{
	brake_state(cur_tac, BRAKING, config.brake_current, "");
}

inline static void manual_brake(const int cur_tac)
{
	brake_state(cur_tac, MANUAL_BRAKING, config.manual_brake_current,
				i >= alive_until ? ", -- Communication timeout, send 'alive <period>'" : "");
}

inline static void pull_state(const int cur_tac, const float pull_current, const skypuff_state new_state,
							  const char *additional_msg)
{
	// Detect direction to zero depending on tachometer value
	float current = cur_tac < 0 ? pull_current : -pull_current;

	state = new_state;

	prev_print = i;
	prev_printed_tac = cur_tac;
	commands_printf("%s: pos: %.2fm (%d steps), pull: %.1fKg (%.1fA)%s",
					state_str(state),
					(double)tac_steps_to_meters(cur_tac), cur_tac,
					(double)(current / config.kg_to_amps), (double)current,
					additional_msg);

	prev_abs_tac = abs(cur_tac);
	prev_erpm = mc_interface_get_rpm();

	smooth_motor_current(cur_tac, prev_abs_tac, current);
	timeout_reset();
}

inline static void unwinding(const int cur_tac)
{
	pull_state(cur_tac, config.unwinding_current, UNWINDING, "");
}

inline static void rewinding(const int cur_tac)
{
	pull_state(cur_tac, config.rewinding_current, REWINDING, "");
}

inline static void pre_pull(const int cur_tac)
{
	// State changed?
	if (state != PRE_PULL)
		pre_pull_start_time = i;

	pull_state(cur_tac, config.pre_pull_k * config.pull_current, PRE_PULL, "");
}

inline static void takeoff_pull(const int cur_tac)
{
	// State changed?
	if (state != TAKEOFF_PULL)
		takeoff_pull_start_time = i;

	pull_state(cur_tac, config.takeoff_pull_k * config.pull_current, TAKEOFF_PULL, "");
}

inline static void pull(const int cur_tac)
{
	pull_state(cur_tac, config.pull_current, PULL, "");
}

inline static void fast_pull(const int cur_tac)
{
	pull_state(cur_tac, config.fast_pull_k * config.pull_current, FAST_PULL, "");
}

inline static void manual_slow_speed_up(const int cur_tac)
{
	char msg[64];
	snprintf(msg, 64, ", until: %.1fms (%.0f ERPM)",
			 (double)erpm_to_ms(config.manual_slow_erpm), (double)config.manual_slow_erpm);
	pull_state(cur_tac, config.manual_slow_speed_up_current, MANUAL_SLOW_SPEED_UP, msg);
}

inline static void manual_slow_back_speed_up(const int cur_tac)
{
	char msg[64];
	snprintf(msg, 64, ", until: %.1fms (%.0f ERPM)",
			 (double)erpm_to_ms(config.manual_slow_erpm), (double)config.manual_slow_erpm);
	pull_state(cur_tac, -config.manual_slow_speed_up_current, MANUAL_SLOW_BACK_SPEED_UP, msg);
}

inline static void slowing(const int cur_tac, const float erpm)
{
	state = SLOWING;

	prev_print = i;
	prev_printed_tac = cur_tac;
	commands_printf("%s: pos: %.2fm (%d steps), speed: %.1fms (%.0f ERPM), until: %.1fms (%.0f ERPM)",
					state_str(state),
					(double)tac_steps_to_meters(cur_tac), cur_tac,
					(double)erpm_to_ms(erpm), (double)erpm,
					(double)erpm_to_ms(config.slow_erpm), (double)config.slow_erpm);

	smooth_motor_release();
}

inline static void slow_state(const int cur_tac, const float cur_erpm,
							  const float constant_erpm, const skypuff_state new_state)
{
	// Detect zero direction depending on tachometer value
	float to_zero_constant_erpm = cur_tac < 0 ? constant_erpm : -constant_erpm;

	state = new_state;

	prev_print = i;
	prev_printed_tac = cur_tac;
	commands_printf("%s: pos: %.2fm (%d steps), speed: %.1fms (%.0f ERPM), constant speed: %.1fms (%.0f ERPM)",
					state_str(state),
					(double)tac_steps_to_meters(cur_tac), cur_tac,
					(double)erpm_to_ms(cur_erpm), (double)cur_erpm,
					(double)erpm_to_ms(to_zero_constant_erpm), (double)to_zero_constant_erpm);

	prev_erpm = cur_erpm;
	smooth_motor_speed(to_zero_constant_erpm);
	timeout_reset();
}

inline static void slow(const int cur_tac, const float cur_erpm)
{
	slow_state(cur_tac, cur_erpm, config.slow_erpm, SLOW);
}

inline static void manual_slow(const int cur_tac, const float cur_erpm)
{
	slow_state(cur_tac, cur_erpm, config.manual_slow_erpm, MANUAL_SLOW);
}

inline static void manual_slow_back(const int cur_tac, const float cur_erpm)
{
	slow_state(cur_tac, cur_erpm, -config.manual_slow_erpm, MANUAL_SLOW_BACK);
}

// Example conf of my winch model: https://youtu.be/KoNegc4SzxY?t=6
static void set_example_conf(skypuff_config *cfg)
{
	// Some example ranges
	cfg->rope_length = meters_to_tac_steps(50);
	cfg->braking_length = meters_to_tac_steps(1);
	cfg->passive_braking_length = meters_to_tac_steps(0.5);
	cfg->overwinding = meters_to_tac_steps(0.1);
	cfg->rewinding_trigger_length = meters_to_tac_steps(0.2);
	cfg->unwinding_trigger_length = meters_to_tac_steps(0.05);
	cfg->slowing_length = meters_to_tac_steps(3);

	// Slow speeds
	cfg->slow_erpm = ms_to_erpm(1);
	cfg->manual_slow_erpm = ms_to_erpm(2);

	// Forces
	cfg->kg_to_amps = 7;   // 7 Amps for 1Kg force
	cfg->amps_per_sec = 2; // Change forces slowly
	cfg->brake_current = 0.3 * set_config.kg_to_amps;
	cfg->manual_brake_current = 1 * set_config.kg_to_amps;
	cfg->unwinding_current = 0.3 * set_config.kg_to_amps;
	cfg->rewinding_current = 0.5 * set_config.kg_to_amps;
	cfg->slow_max_current = 0.4 * set_config.kg_to_amps;
	cfg->manual_slow_max_current = 0.4 * set_config.kg_to_amps;
	cfg->manual_slow_speed_up_current = 0.3 * set_config.kg_to_amps;

	// Pull settings
	cfg->pull_current = 0.5 * set_config.kg_to_amps;
	cfg->pre_pull_k = 30 / 100.0;
	cfg->takeoff_pull_k = 60 / 100.0;
	cfg->fast_pull_k = 120 / 100.0;
	cfg->takeoff_trigger_length = meters_to_tac_steps(0.1);
	cfg->pre_pull_timeout = 2 * 1000;
	cfg->takeoff_period = 5 * 1000;
}

// Called when the custom application is started. Start our
// threads here and set up callbacks.
void app_custom_start(void)
{
	commands_printf("app_skypuff started");

	// Reset tachometer on app start to prevent unwinding to zero
	mc_interface_get_tachometer_value(true);

	mc_conf = mc_interface_get_configuration();

	prev_print = INT_MIN / 2;
	prev_printed_tac = INT_MIN / 2;
	alive_until = 0;

	smooth_motor_release();

	state = is_config_out_of_limits(&config) ? UNINITIALIZED : BRAKING;
	terminal_command = DO_NOTHING;

	stop_now = false;

	// Terminal commands for the VESC Tool terminal can be registered.
	terminal_register_command_callback(
		"set_zero",
		"Move zero point here.",
		"", terminal_set_zero);
	terminal_register_command_callback(
		"skypuff",
		"Print SkyPUFF configuration here.",
		"", terminal_print_conf);
	terminal_register_command_callback(
		"example_conf",
		"Set SkyPUFF model winch configuration.",
		"", terminal_set_example_conf);
	terminal_register_command_callback(
		"alive",
		"Prolong communication alive period.",
		"[milliseconds]", terminal_alive);
	terminal_register_command_callback(
		"set",
		"Set new SkyPUFF state: MANUAL_BRAKING, UNWINDING, MANUAL_SLOW, MANUAL_SLOW_BACK, PRE_PULL, TAKEOFF_PULL, PULL, FAST_PULL.",
		"state", terminal_set_state);
	terminal_register_command_callback(
		"force",
		"Set pull force.",
		"[kg]", terminal_set_pull_force);
#ifdef DEBUG_SMOOTH_MOTOR
	terminal_register_command_callback(
		"smooth",
		"Debug smooth motor control.",
		"<release/brake/current/speed> [current/erpm]", terminal_smooth);
#endif

	// Run control loop thread
	chThdCreateStatic(my_thread_wa, sizeof(my_thread_wa), NORMALPRIO, my_thread, NULL);
}

// Called when the custom application is stopped. Stop our threads
// and release callbacks.
void app_custom_stop(void)
{
	terminal_unregister_callback(terminal_set_zero);
	terminal_unregister_callback(terminal_print_conf);
	terminal_unregister_callback(terminal_set_example_conf);
	terminal_unregister_callback(terminal_alive);
	terminal_unregister_callback(terminal_set_state);
	terminal_unregister_callback(terminal_set_pull_force);
#ifdef DEBUG_SMOOTH_MOTOR
	terminal_unregister_callback(terminal_smooth);
#endif

	stop_now = true;
	while (is_running)
	{
		chThdSleepMilliseconds(1);
	}
	commands_printf("app_skypuff stopped");
}

void app_custom_configure(app_configuration *conf)
{
	(void)conf;
}

// The same code for unwinding and rewinding states
// Returns true if mode changed
static bool brake_or_slowing(const int cur_tac, const int abs_tac)
{
	// We are in the braking range with overwinding?
	if (abs_tac < config.braking_length - config.overwinding)
	{
		brake(cur_tac);
		return true;
	}

	// We are in the slowing range?
	if (abs_tac < config.braking_length + config.slowing_length)
	{
		float erpm = mc_interface_get_rpm();

		// Rewinding forward with more then slow speed?
		if (cur_tac < 0 && erpm > config.slow_erpm)
		{
			slowing(cur_tac, erpm);
			return true;
		}
		// Rewinding backward with more then slow speed?
		else if (cur_tac >= 0 && erpm < -config.slow_erpm)
		{
			slowing(cur_tac, erpm);
			return true;
		}
	}

	return false;
}

inline static void print_position_periodically(const int cur_tac, const int delay, const char *additional_msg)
{
	// prolong delay if not moving
	if (cur_tac == prev_printed_tac)
	{
		prev_print = i;
		return;
	}

	if (i - prev_print > delay)
	{
		prev_print = i;
		prev_printed_tac = cur_tac;
		commands_printf("%s: pos %.2fm (%d steps)%s",
						state_str(state),
						(double)tac_steps_to_meters(cur_tac), cur_tac,
						additional_msg);
	}
}

inline static bool is_direction_changed_or_stopped(const float cur_erpm)
{
	float tmp_erpm = prev_erpm;
	prev_erpm = cur_erpm;

	// Rotating direction changed or stopped?
	if ((tmp_erpm < -1 && cur_erpm >= 0) || (tmp_erpm > 1 && cur_erpm <= 0))
	{
		commands_printf("%s: -- rotation direction changed, prev_erpm: %.1fms (%0.f ERPM), cur_erpm: %.1fms (%0.f ERPM)",
						state_str(state),
						(double)erpm_to_ms(tmp_erpm), (double)tmp_erpm,
						(double)erpm_to_ms(cur_erpm), (double)cur_erpm);
		return true;
	}

	return false;
}

inline static void brake_or_unwinding(const int cur_tac, const int abs_tac)
{
	if (abs_tac < config.braking_length - config.overwinding)
		brake(cur_tac);
	else
		unwinding(cur_tac);
}

inline static void brake_or_manual_brake(const int cur_tac, const int abs_tac)
{
	if (abs_tac < config.braking_length - config.overwinding)
		brake(cur_tac);
	else
		manual_brake(cur_tac);
}

inline static void slowing_or_speed_up_print(const int cur_tac, const float cur_erpm, const float target_erpm)
{
	if (i - prev_print > short_print_delay)
	{
		int overwinding = config.braking_length - config.overwinding;
		int distance_left = abs(cur_tac) - overwinding;
		prev_print = i;
		commands_printf(
			"%s, pos %.2fm (%d steps), speed: %.1fms (%.0f ERPM), until: %.1fms (%.0f ERPM), -- %.2fm to overwinding zone",
			state_str(state),
			(double)tac_steps_to_meters(cur_tac), cur_tac,
			(double)erpm_to_ms(cur_erpm), (double)cur_erpm,
			(double)erpm_to_ms(target_erpm), (double)target_erpm,
			(double)tac_steps_to_meters(distance_left));
	}
}

inline static void state_switch(const int cur_tac, const int abs_tac)
{
	float cur_erpm, abs_erpm;
	float cur_current, abs_current;

	switch (state)
	{
	case UNINITIALIZED:
		// Only correct SET_CONF will move us from here
		print_position_periodically(cur_tac, long_print_delay,
									abs_tac > config.braking_length ? ", -- Out of safe braking zone!" : "");
		break;
	case BRAKING:
		// We are in the breaking + passive zone?
		if (abs_tac <= config.braking_length + config.passive_braking_length)
		{
			// Timeout thread will remove braking every second by default
			// Brake again if position changed
			if (timeout_has_timeout() && abs_tac != prev_abs_tac)
				brake(cur_tac);
			else
				prev_abs_tac = abs_tac;
		}
		else
			unwinding(cur_tac);

		print_position_periodically(cur_tac, long_print_delay, "");
		break;
	case MANUAL_BRAKING:
		if (timeout_has_timeout() && abs_tac != prev_abs_tac)
			manual_brake(cur_tac);
		else
			prev_abs_tac = abs_tac;

		print_position_periodically(cur_tac, long_print_delay,
									i >= alive_until ? ", -- Communication timeout" : "");

		break;
	case UNWINDING:
		// No timeouts for unwinding state
		if (!(i % 500))
			timeout_reset();

		// Go breaking or slowing?
		if (brake_or_slowing(cur_tac, abs_tac))
			break;

		// Use prev_abs_tac as max tachometer
		if (abs_tac > prev_abs_tac)
		{
			// Print debug message if we are going out from slowing zone
			int eof_slowing = config.braking_length + config.slowing_length;
			if (prev_abs_tac < eof_slowing && abs_tac >= eof_slowing)
			{
				commands_printf("%s: -- Unwinded from slowing zone %.2fm (%d steps)",
								state_str(state),
								(double)tac_steps_to_meters(cur_tac), cur_tac);
			}

			// Update maximum value of tachometer
			prev_abs_tac = abs_tac;
		}

		// Going back more then config.rewinding_trigger_length?
		if (abs_tac < prev_abs_tac - config.rewinding_trigger_length)
		{
			rewinding(cur_tac);
			break;
		}

		print_position_periodically(cur_tac, long_print_delay, "");
		break;
	case REWINDING:
		// No timeouts for rewinding state
		if (!(i % 500))
			timeout_reset();

		// Go breaking or slowing?
		if (brake_or_slowing(cur_tac, abs_tac))
			break;

		// Now use prev_abs_tac as min value
		if (abs_tac < prev_abs_tac)
			prev_abs_tac = abs_tac;

		// Unwinding again?
		if (abs_tac > prev_abs_tac + config.unwinding_trigger_length)
			unwinding(cur_tac);

		print_position_periodically(cur_tac, long_print_delay, "");
		break;
	case SLOWING:
		// We are in the braking range with overwinding?
		if (abs_tac < config.braking_length - config.overwinding)
		{
			brake(cur_tac);
			break;
		}

		cur_erpm = mc_interface_get_rpm();
		abs_erpm = fabs(cur_erpm);

		// Slow enough for PID speed?
		if (abs_erpm < config.slow_erpm)
		{
			slow(cur_tac, cur_erpm);
			break;
		}

		slowing_or_speed_up_print(cur_tac, cur_erpm, config.slow_erpm);
		break;
	case SLOW:
		cur_current = mc_interface_get_tot_current_directional_filtered();
		abs_current = fabs(cur_current);
		cur_erpm = mc_interface_get_rpm();

		// No timeouts for slow state
		if (!(i % 500))
			timeout_reset();

		// Rotating direction changed or stopped?
		if (is_direction_changed_or_stopped(cur_erpm))
		{
			brake_or_unwinding(cur_tac, abs_tac);
			break;
		}

		// If current above the limits - brake or unwinding
		if (abs_current > config.slow_max_current)
		{
			commands_printf(
				"SLOW: speed %.1fms (%.0f ERPM), -- slow pulling too high %.1fKg (%.1fA) is more %.1fKg (%.1fA)",
				(double)erpm_to_ms(cur_erpm), (double)cur_erpm,
				(double)(cur_current / config.kg_to_amps), (double)cur_current,
				(double)(config.slow_max_current / config.kg_to_amps), (double)config.slow_max_current);

			brake_or_unwinding(cur_tac, abs_tac);
			break;
		}

		// Slowly rewinded more then opposite side of (breaking - overwinding) zone?
		if ((cur_erpm > 0 && cur_tac >= config.braking_length - config.overwinding) ||
			(cur_erpm < 0 && cur_tac <= -config.braking_length + config.overwinding))
		{
			commands_printf("SLOW: -- winded to opposite braking zone");
			brake(cur_tac);
			break;
		}

		if (i - prev_print > long_print_delay)
		{
			prev_print = i;
			commands_printf("SLOW: pos %.2fm (%d steps), speed %.1fms (%.0f ERPM), pull %.1fKg (%.1fA)",
							(double)tac_steps_to_meters(cur_tac), cur_tac,
							(double)erpm_to_ms(cur_erpm), (double)cur_erpm,
							(double)(cur_current / config.kg_to_amps), (double)cur_current);
		}
		break;
	case MANUAL_SLOW_SPEED_UP:
		cur_erpm = mc_interface_get_rpm();
		abs_erpm = fabs(cur_erpm);

		// No timeouts for manual_slow_speed_up state
		if (!(i % 500))
			timeout_reset();

		// Do not rotate in braking zone
		if (abs_tac < config.braking_length - config.overwinding)
		{
			commands_printf("%s: speed %.1fms (%.0f ERPM) -- Braking zone",
							(double)erpm_to_ms(cur_erpm), (double)cur_erpm, state_str(state));
			brake(cur_tac);
			break;
		}

		// Rotating direction changed or stopped?
		if (is_direction_changed_or_stopped(cur_erpm))
		{
			manual_brake(cur_tac);
			break;
		}

		// Fast enough for PID speed?
		if (abs_erpm >= config.manual_slow_erpm)
		{
			manual_slow(cur_tac, cur_erpm);
			break;
		}

		slowing_or_speed_up_print(cur_tac, cur_erpm, config.manual_slow_erpm);

		break;
	case MANUAL_SLOW_BACK_SPEED_UP:
		cur_erpm = mc_interface_get_rpm();
		abs_erpm = fabs(cur_erpm);

		// No timeouts for manual_slow_speed_up state
		if (!(i % 500))
			timeout_reset();

		// Rotating direction changed or stopped?
		if (is_direction_changed_or_stopped(cur_erpm))
		{
			manual_brake(cur_tac);
			break;
		}

		// Fast enough for PID speed?
		if (abs_erpm > config.manual_slow_erpm)
		{
			manual_slow_back(cur_tac, cur_erpm);
			break;
		}

		slowing_or_speed_up_print(cur_tac, cur_erpm, config.manual_slow_erpm);

		break;
	case MANUAL_SLOW:
		cur_current = mc_interface_get_tot_current_directional_filtered();
		abs_current = fabs(cur_current);
		cur_erpm = mc_interface_get_rpm();

		// No timeouts for manual_slow state
		if (!(i % 500))
			timeout_reset();

		// If slowing zone and speed is more then SLOW, go SLOWING
		if (abs_tac < config.braking_length + config.slowing_length &&
			config.slow_erpm < config.manual_slow_erpm)
		{
			commands_printf("%s: -- Slowing zone, manual slow speed is too high, go SLOWING", state_str(state));
			slowing(cur_tac, cur_erpm);
			break;
		}

		// Rotating direction changed or stopped?
		if (is_direction_changed_or_stopped(cur_erpm))
		{
			brake_or_manual_brake(cur_tac, abs_tac);
			break;
		}

		// If current above the limits - brake or unwinding
		if (abs_current > config.manual_slow_max_current)
		{
			commands_printf(
				"MANUAL_SLOW: speed %.1fms (%.0f ERPM), -- slow pulling too high %.1fKg (%.1fA) is more %.1fKg (%.1fA)",
				(double)erpm_to_ms(cur_erpm), (double)cur_erpm,
				(double)(cur_current / config.kg_to_amps), (double)cur_current,
				(double)(config.manual_slow_max_current / config.kg_to_amps), (double)config.manual_slow_max_current);

			brake_or_manual_brake(cur_tac, abs_tac);
			break;
		}

		// Slowly rewinded more then opposite side of (breaking - overwinding) zone?
		if ((cur_erpm > 0 && cur_tac >= config.braking_length - config.overwinding) ||
			(cur_erpm < 0 && cur_tac <= -config.braking_length + config.overwinding))
		{
			commands_printf("MANUAL_SLOW: -- winded to opposite braking zone");
			brake(cur_tac);
			break;
		}

		if (i - prev_print > long_print_delay)
		{
			prev_print = i;
			commands_printf("MANUAL_SLOW: pos %.2fm (%d steps), speed: %.1fms (%.0f ERPM), pull: %.1fKg (%.1fA)",
							(double)tac_steps_to_meters(cur_tac), cur_tac,
							(double)erpm_to_ms(cur_erpm), (double)cur_erpm,
							(double)(cur_current / config.kg_to_amps), (double)cur_current);
		}

		break;
	case MANUAL_SLOW_BACK:
		cur_current = mc_interface_get_tot_current_directional_filtered();
		abs_current = fabs(cur_current);
		cur_erpm = mc_interface_get_rpm();

		// No timeouts for manual_slow state
		if (!(i % 500))
			timeout_reset();

		// Rotating direction changed or stopped?
		if (is_direction_changed_or_stopped(cur_erpm))
		{
			brake_or_manual_brake(cur_tac, abs_tac);
			break;
		}

		// If current above the limits - brake or unwinding
		if (abs_current > config.manual_slow_max_current)
		{
			commands_printf(
				"MANUAL_SLOW_BACK: speed %.1fms (%.0f ERPM), -- slow pulling too high %.1fKg (%.1fA) is more %.1fKg (%.1fA)",
				(double)erpm_to_ms(cur_erpm), (double)cur_erpm,
				(double)(cur_current / config.kg_to_amps), (double)cur_current,
				(double)(config.manual_slow_max_current / config.kg_to_amps),
				(double)config.manual_slow_max_current);

			brake_or_manual_brake(cur_tac, abs_tac);
			break;
		}

		if (i - prev_print > long_print_delay)
		{
			prev_print = i;
			commands_printf("MANUAL_SLOW_BACK: pos %.2fm (%d steps), speed: %.1fms (%.0f ERPM), pull: %.1fKg (%.1fA)",
							(double)tac_steps_to_meters(cur_tac), cur_tac,
							(double)erpm_to_ms(cur_erpm), (double)cur_erpm,
							(double)(cur_current / config.kg_to_amps), (double)cur_current);
		}

		break;
	case PRE_PULL:
		// No timeouts for pre pull state
		if (!(i % 500))
			timeout_reset();

		// Go breaking or slowing?
		if (brake_or_slowing(cur_tac, abs_tac))
			break;

		// Enough time for the rope tighten?
		if (i == pre_pull_start_time + config.pre_pull_timeout)
		{
			commands_printf("%s: pos %.2fm (%d steps), -- Pre pull %.1fs timeout passed, saving position",
							state_str(state),
							(double)tac_steps_to_meters(cur_tac), cur_tac,
							(double)config.pre_pull_timeout / (double)1000.0);
			prev_abs_tac = abs_tac;
		}

		// Moved enough to takeoff?
		if (abs(prev_abs_tac - abs_tac) >= config.takeoff_trigger_length)
		{
			commands_printf("%s: pos %.2fm (%d steps), moved %.2fm (%d steps) -- Moving detected",
							state_str(state),
							(double)tac_steps_to_meters(cur_tac), cur_tac,
							(double)tac_steps_to_meters(config.takeoff_trigger_length), config.takeoff_trigger_length);
			takeoff_pull(cur_tac);
			break;
		}

		print_position_periodically(cur_tac, long_print_delay, "");
		break;
	case TAKEOFF_PULL:
		// No timeouts for takeoff pull state
		if (!(i % 500))
			timeout_reset();

		// Go breaking or slowing?
		if (brake_or_slowing(cur_tac, abs_tac))
			break;

		//
		if (i >= takeoff_pull_start_time + config.takeoff_period)
		{
			commands_printf("%s: pos %.2fm (%d steps), -- Takeoff %.1fs timeout passed",
							state_str(state),
							(double)tac_steps_to_meters(cur_tac), cur_tac,
							(double)config.takeoff_period / (double)1000.0);
			pull(cur_tac);
			break;
		}
		print_position_periodically(cur_tac, long_print_delay, "");
		break;
	case PULL:
		// No timeouts for pull state
		if (!(i % 500))
			timeout_reset();

		// Go breaking or slowing?
		if (brake_or_slowing(cur_tac, abs_tac))
			break;

		print_position_periodically(cur_tac, long_print_delay, "");
		break;
	case FAST_PULL:
		// No timeouts for fast pull state
		if (!(i % 500))
			timeout_reset();

		// Go breaking or slowing?
		if (brake_or_slowing(cur_tac, abs_tac))
			break;

		print_position_periodically(cur_tac, long_print_delay, "");
		break;
#ifdef DEBUG_SMOOTH_MOTOR
	case MANUAL_DEBUG_SMOOTH:
		// No timeouts for fast pull state
		if (!(i % 500))
			timeout_reset();

		print_position_periodically(cur_tac, long_print_delay, "");
		break;
#endif
	default:
		commands_printf("SkyPUFF: unknown control loop state, exiting!");
		stop_now = true;
	}
}

inline static void print_conf(const int cur_tac)
{
	commands_printf("VESC additional info:");
	commands_printf("  wheel diameter: %.2fmm", (double)(mc_conf->si_wheel_diameter * 1000));
	commands_printf("  motor poles: %dp", mc_conf->si_motor_poles);
	commands_printf("  gear ratio: %.5f", (double)mc_conf->si_gear_ratio);

	commands_printf("SkyPUFF configuration v%d:", config_version);
	commands_printf("  amperes per 1kg force: %.1fAKg", (double)config.kg_to_amps);
	commands_printf("  force changing speed: %.2fKg/sec (%.1fA/sec)", (double)(config.amps_per_sec / config.kg_to_amps), (double)config.amps_per_sec);
	commands_printf("  rope length: %.2fm (%d steps)", (double)tac_steps_to_meters(config.rope_length), config.rope_length);
	commands_printf("  braking range: %.2fm (%d steps)", (double)tac_steps_to_meters(config.braking_length), config.braking_length);
	commands_printf("  passive braking range: %.2fm (%d steps)", (double)tac_steps_to_meters(config.passive_braking_length), config.passive_braking_length);
	commands_printf("  overwinding: %.2fm (%d steps)", (double)tac_steps_to_meters(config.overwinding), config.overwinding);
	commands_printf("  slowing range: %.2fm (%d steps)", (double)tac_steps_to_meters(config.slowing_length), config.slowing_length);
	commands_printf("  rewinding trigger range: %.2fm (%d steps)", (double)tac_steps_to_meters(config.rewinding_trigger_length), config.rewinding_trigger_length);
	commands_printf("  unwinding trigger range: %.2fm (%d steps)", (double)tac_steps_to_meters(config.unwinding_trigger_length), config.unwinding_trigger_length);
	commands_printf("  brake force: %.2fkg (%.1fA)", (double)(config.brake_current / config.kg_to_amps), (double)config.brake_current);
	commands_printf("  manual brake force: %.2fkg (%.1fA)", (double)(config.manual_brake_current / config.kg_to_amps), (double)config.manual_brake_current);
	commands_printf("  unwinding force: %.2fkg (%.1fA)", (double)(config.unwinding_current / config.kg_to_amps), (double)config.unwinding_current);
	commands_printf("  rewinding force: %.2fkg (%.1fA)", (double)(config.rewinding_current / config.kg_to_amps), (double)config.rewinding_current);
	commands_printf("  slow speed: %.1fms (%.0f ERPM)", (double)erpm_to_ms(config.slow_erpm), (double)config.slow_erpm);
	commands_printf("  maximum slow force: %.2fkg (%.1fA)", (double)(config.slow_max_current / config.kg_to_amps), (double)config.slow_max_current);
	commands_printf("  manual slow max force: %.2fkg (%.1fA)", (double)(config.manual_slow_max_current / config.kg_to_amps), (double)config.manual_slow_max_current);
	commands_printf("  manual slow speed up force: %.2fkg (%.1fA)", (double)(config.manual_slow_speed_up_current / config.kg_to_amps), (double)config.manual_slow_speed_up_current);
	commands_printf("  manual slow speed: %.1fms (%.0f ERPM)", (double)erpm_to_ms(config.manual_slow_erpm), (double)config.manual_slow_erpm);

	commands_printf("  pull force: %.2fkg (%.1fA)", (double)(config.pull_current / config.kg_to_amps), (double)config.pull_current);
	commands_printf("  takeoff trigger range: %.2fm (%d steps)", (double)tac_steps_to_meters(config.takeoff_trigger_length), config.takeoff_trigger_length);
	commands_printf("  pre pull timeout: %.1fs (%d loops)", (double)config.pre_pull_timeout / (double)1000.0, config.pre_pull_timeout);
	commands_printf("  takeoff period: %.1fs (%d loops)", (double)config.takeoff_period / (double)1000.0, config.takeoff_period);
	commands_printf("  pre pull percent: %.0f%% (%.2fkg, %.5f)", (double)config.pre_pull_k * (double)100.0, (double)(config.pull_current / config.kg_to_amps * config.pre_pull_k), (double)config.pre_pull_k);
	commands_printf("  takeoff pull percent: %.0f%% (%.2fkg, %.5f)", (double)config.takeoff_pull_k * (double)100.0, (double)(config.pull_current / config.kg_to_amps * config.takeoff_pull_k), (double)config.takeoff_pull_k);
	commands_printf("  fast pull percent: %.0f%% (%.2fkg, %.5f)", (double)config.fast_pull_k * (double)100.0, (double)(config.pull_current / config.kg_to_amps * config.fast_pull_k), (double)config.fast_pull_k);

	commands_printf("SkyPUFF state:");
	commands_printf("  state %s, current position: %.2fm (%d steps)", state_str(state), (double)tac_steps_to_meters(cur_tac), cur_tac);
	commands_printf("  loop counter: %d, alive until: %d, %s", i, alive_until, i >= alive_until ? "communication timeout" : "no timeout");
}

inline static void terminal_command_switch(const int cur_tac, const int abs_tac)
{
	// In case of new command during next switch
	skypuff_terminal_command prev_command = terminal_command;

	switch (terminal_command)
	{
	case DO_NOTHING:
		return;

	case SET_ZERO:
		switch (state)
		{
		case UNINITIALIZED:
		case BRAKING:
		case MANUAL_BRAKING:
			mc_interface_get_tachometer_value(true);
			prev_abs_tac = 0;
			commands_printf("%s: -- Zero is set", state_str(state));
			break;

		default:
			commands_printf("%s: -- Zero could be set from UNITIALIZED, BRAKING or MANUAL_BRAKING",
							state_str(state));
			break;
		}
		break;
	case SET_MANUAL_BRAKING:
		if (state != UNINITIALIZED)
			manual_brake(cur_tac);
		else
			commands_printf("%s: -- Can't switch to MANUAL_BRAKING from UNITIALIZED", state_str(state));

		break;
	case SET_MANUAL_SLOW:
		if (abs_tac <= config.braking_length)
			commands_printf("%s: -- MANUAL_SLOW could be only set after braking zone %.1fm (%d steps)",
							state_str(state), (double)tac_steps_to_meters(config.braking_length),
							config.braking_length);
		else if (state == MANUAL_BRAKING)
			manual_slow_speed_up(cur_tac);
		else
			commands_printf("%s: -- MANUAL_SLOW transition only possible from MANUAL_BRAKING", state_str(state));

		break;
	case SET_MANUAL_SLOW_BACK:
		if (abs_tac <= config.braking_length)
			commands_printf("%s: -- MANUAL_SLOW_BACK could be only set after braking zone %.1fm (%d steps)",
							state_str(state), (double)tac_steps_to_meters(config.braking_length),
							config.braking_length);
		else if (state == MANUAL_BRAKING)
			manual_slow_back_speed_up(cur_tac);
		else
			commands_printf("%s: -- MANUAL_SLOW_BACK transition only possible from MANUAL_BRAKING", state_str(state));

		break;
	case SET_UNWINDING:
		// Timeout?
		if (i >= alive_until)
		{
			commands_printf("%s: -- Update timeout with 'alive <period>' before switching to UNWINDING",
							state_str(state));
			break;
		}

		// Possible from manual braking or pulling states
		switch (state)
		{
		case MANUAL_BRAKING:
		case PRE_PULL:
		case TAKEOFF_PULL:
		case PULL:
		case FAST_PULL:
			unwinding(cur_tac);
			break;

		default:
			commands_printf("%s: -- Switch to UNWINDING only possible from MANUAL_BRAKING, PRE_PULL, TAKEOFF_PULL, PULL, FAST_PULL", state_str(state));
			break;
		}

		break;
	case SET_PULL_FORCE:
		// We need correct config.kg_to_amps and drive settings
		if (state == UNINITIALIZED)
		{
			commands_printf("%s: -- Can't update pull force from UNITIALIZED", state_str(state));
			break;
		}

		// Calculate amperes from Kg
		float pull_current = config.kg_to_amps * terminal_pull_kg;

		if (is_pull_out_of_limits("pull_current",
								  pull_current,
								  min_config.pull_current,
								  max_config.pull_current))
			break;

		config.pull_current = pull_current;

		// Update pull force now?
		switch (state)
		{
		case PRE_PULL:
			pre_pull(cur_tac);
			break;
		case TAKEOFF_PULL:
			takeoff_pull(cur_tac);
			break;
		case PULL:
			pull(cur_tac);
			break;
		case FAST_PULL:
			fast_pull(cur_tac);
			break;

		default:
			break;
		}

		break;
	case SET_PRE_PULL:
		switch (state)
		{
		case UNWINDING:
		case REWINDING:
			pre_pull(cur_tac);
			break;
		default:
			commands_printf("%s: -- PRE_PULL could only be set from UNWINDING or REWINDING", state_str(state));
			break;
		}

		break;
	case SET_TAKEOFF_PULL:
		if (state != PRE_PULL)
		{
			commands_printf("%s: -- TAKEOFF_PULL could only be set from PRE_PULL", state_str(state));
			break;
		}

		takeoff_pull(cur_tac);

		break;
	case SET_PULL:
		switch (state)
		{
		case TAKEOFF_PULL:
		case UNWINDING:
		case REWINDING:
			pull(cur_tac);
			break;

		default:
			commands_printf("%s: -- PULL could only be set from TAKEOFF_PULL or UNWINDING", state_str(state));
		}

		break;
	case SET_FAST_PULL:
		if (state != PULL)
		{
			commands_printf("%s: -- FAST_PULL could only be set from PULL", state_str(state));
			break;
		}

		fast_pull(cur_tac);

		break;
	case GET_CONF:
		print_conf(cur_tac);

		break;
	case SET_CONF:
		switch (state)
		{
		case UNINITIALIZED:
		case BRAKING:
			if (is_config_out_of_limits(&set_config))
				break;

			// Use braking_length from received config
			if (abs_tac > set_config.braking_length)
			{
				commands_printf("%s: -- Can't set configuration when position is out of braking zone", state_str(state));
				break;
			}

			config = set_config;
			commands_printf("%s: -- Configuration set", state_str(state));

			if (state == UNINITIALIZED)
				brake(cur_tac);

			break;
		default:
			commands_printf("%s: -- Configuration could be updated in UNITIALIZED or BRAKING states",
							state_str(state));
		}

		break;
#ifdef DEBUG_SMOOTH_MOTOR
	case SET_SMOOTH:
		state = MANUAL_DEBUG_SMOOTH;

		switch (terminal_motor_state.mode)
		{
		case MOTOR_RELEASED:
			smooth_motor_release();
			break;
		case MOTOR_BRAKING:
			smooth_motor_brake(cur_tac, abs_tac, terminal_motor_state.param.current);
			break;
		case MOTOR_CURRENT:
			smooth_motor_current(cur_tac, abs_tac, terminal_motor_state.param.current);
			break;
		case MOTOR_SPEED:
			smooth_motor_speed(terminal_motor_state.param.erpm);
			break;

		default:
			break;
		}

		break;
#endif
	default:
		commands_printf("SkyPUFF: unknown terminal command, exiting!");
		stop_now = true;
	}

	// Nothing changed during switch?
	if (prev_command == terminal_command)
		terminal_command = DO_NOTHING;
}

static THD_FUNCTION(my_thread, arg)
{
	(void)arg;

	chRegSetThreadName("App SkyPUFF");

	is_running = true;

	for (i = 0;; i++)
	{
		// Check if it is time to stop.
		if (stop_now)
		{
			is_running = false;
			return;
		}

		int cur_tac = mc_interface_get_tachometer_value(false);
		int abs_tac = abs(cur_tac);

		// terminal command 'alive'?
		if (alive_inc)
		{
			alive_until = i + alive_inc;
			alive_inc = 0;
			// Actually not necessary
			// commands_printf("%s: time %d, until %d",
			// 				state_str(state), i, alive_until);
		}

		// Communication timeout, initialized and not braking zone?
		if (i >= alive_until && abs_tac > config.braking_length &&
			state != UNINITIALIZED && state != MANUAL_BRAKING)
			manual_brake(cur_tac);

		terminal_command_switch(cur_tac, abs_tac);
		state_switch(cur_tac, abs_tac);

		if (i >= next_smooth_motor_adjustment)
			smooth_motor_adjustment(cur_tac, abs_tac);

		chThdSleepMilliseconds(1);
	}
}

// Terminal command to change tachometer value
// NOT SAFE
/*
static void terminal_move_tac(int argc, const char **argv)
{
	if (argc == 2)
	{
		float d = 0;
		if (sscanf(argv[1], "%f", &d) == EOF)
		{
			commands_printf("move_tac: can't parse meters: '%s' value.", argv[1]);
			return;
		};

		int steps = meters_to_tac_steps(d);

		int cur_tac = mc_interface_get_tachometer_value(false);

		int new_tac = cur_tac + steps;

		commands_printf("move_tac: moving zero %.2fm (%d steps) %s, cur pos: %.2fm (%d steps), new pos: %.2fm (%d steps)",
						(double)d, steps, d < 0 ? "backward" : "forward",
						(double)tac_steps_to_meters(cur_tac), cur_tac,
						(double)tac_steps_to_meters(new_tac), new_tac);

		mc_interface_set_tachometer_value(new_tac);
	}
	else
	{
		commands_printf("This command requires one argument: 'move_tac -5.2' will move zero 5.2 meters backward");
	}
}
*/

static void terminal_set_zero(int argc, const char **argv)
{
	(void)argc;
	(void)argv;

	terminal_command = SET_ZERO;
}

static void terminal_print_conf(int argc, const char **argv)
{
	(void)argc;
	(void)argv;

	terminal_command = GET_CONF;
}

static void terminal_set_example_conf(int argc, const char **argv)
{
	(void)argc;
	(void)argv;

	set_example_conf(&set_config);
	terminal_command = SET_CONF;
}

static void terminal_alive(int argc, const char **argv)
{
	if (argc < 2)
	{
		commands_printf("%s: -- Command requires one argument -- 'alive 300' will let skypuff work 300ms before timeout and MANUAL_BRAKING");
		return;
	}

	int t = 0;
	if (sscanf(argv[1], "%d", &t) == EOF)
	{
		commands_printf("%s: -- Can't parse '%s' as milliseconds value.",
						state_str(state), argv[1]);
		return;
	};

	alive_inc = t;
}

static void terminal_set_pull_force(int argc, const char **argv)
{
	if (argc < 2)
	{
		commands_printf("%s: -- Command requires one argument -- 'pull_force 100' will set pull force 100Kg");
		return;
	}

	float kg;
	if (sscanf(argv[1], "%f", &kg) == EOF)
	{
		commands_printf("%s: -- Can't parse '%s' as kg value.",
						state_str(state), argv[1]);
		return;
	};

	// Better check limits in the terminal_command_switch
	/*
	if (kg <= min_pull_kg || kg >= max_pull_kg)
	{
		commands_printf("%s: -- Pull force %.1fKg out of limits (%.1f, %.1f)",
						state_str(state), (double)kg, (double)min_pull_kg, (double)max_pull_kg);
	}
	*/

	terminal_pull_kg = kg;
	terminal_command = SET_PULL_FORCE;
}

inline static void uppercase(char *out, const char *in, const int out_len)
{
	int in_len = strlen(in);

	// do not crash MCU occasionally
	if (out_len <= 0 || out_len > 1024)
		return;

	// Get minumum
	int valid_len = out_len - 1 < in_len ? out_len - 1 : in_len;

	// Iterate over the source string (i.e. s) and cast the case changing.
	for (int a = 0; a < valid_len; a++)
		out[a] = toupper(in[a]);

	// The last zero
	out[valid_len] = 0;
}

static void terminal_set_state(int argc, const char **argv)
{
	if (argc < 2)
	{
		commands_printf("%s: -- Command requires at least one argument -- For example: 'set_state UNWINDING'",
						state_str(state));
		return;
	}

	const int up_len = 64;
	char up[up_len];
	uppercase(up, argv[1], up_len);

	if (!strcmp(up, "UNWINDING"))
	{
		terminal_command = SET_UNWINDING;
		return;
	}
	else if (!strcmp(up, "MANUAL_BRAKING"))
	{
		terminal_command = SET_MANUAL_BRAKING;
		return;
	}
	else if (!strcmp(up, "MANUAL_SLOW"))
	{
		terminal_command = SET_MANUAL_SLOW;
		return;
	}
	else if (!strcmp(up, "MANUAL_SLOW_BACK"))
	{
		terminal_command = SET_MANUAL_SLOW_BACK;
		return;
	}
	else if (!strcmp(up, "PRE_PULL"))
	{
		terminal_command = SET_PRE_PULL;
		return;
	}
	else if (!strcmp(up, "TAKEOFF_PULL"))
	{
		terminal_command = SET_TAKEOFF_PULL;
		return;
	}
	else if (!strcmp(up, "PULL"))
	{
		terminal_command = SET_PULL;
		return;
	}
	else if (!strcmp(up, "FAST_PULL"))
	{
		terminal_command = SET_FAST_PULL;
		return;
	}
	else
	{
		commands_printf("%s: -- 'set_state %s' not implemented",
						state_str(state), argv[1]);
	}
}

#ifdef DEBUG_SMOOTH_MOTOR
static void terminal_smooth(int argc, const char **argv)
{
	if (argc < 2)
	{
		commands_printf("%s: -- Command requires at least one argument -- For example: 'smooth RELEASE'",
						state_str(state));
		return;
	}

	const int up_len = 64;
	char up[up_len];
	uppercase(up, argv[1], up_len);

	if (!strcmp(up, "RELEASE"))
	{
		terminal_command = SET_SMOOTH;
		terminal_motor_state.mode = MOTOR_RELEASED;
		terminal_motor_state.param.current = 0;
		return;
	}
	else if (!strcmp(up, "BRAKE"))
	{
		if (argc < 3)
		{
			commands_printf("%s: -- smooth BRAKE requires current argument -- For example: 'smooth BRAKE 2'",
							state_str(state));
			return;
		}
		float current;
		if (sscanf(argv[2], "%f", &current) == EOF)
		{
			commands_printf("%s: -- Can't parse '%s' as current value.",
							state_str(state), argv[2]);
			return;
		};
		terminal_command = SET_SMOOTH;
		terminal_motor_state.mode = MOTOR_BRAKING;
		terminal_motor_state.param.current = current;
		return;
	}
	else if (!strcmp(up, "CURRENT"))
	{
		if (argc < 3)
		{
			commands_printf("%s: -- smooth CURRENT requires current argument -- For example: 'smooth CURRENT 2'",
							state_str(state));
			return;
		}
		float current;
		if (sscanf(argv[2], "%f", &current) == EOF)
		{
			commands_printf("%s: -- Can't parse '%s' as current value.",
							state_str(state), argv[2]);
			return;
		};
		terminal_command = SET_SMOOTH;
		terminal_motor_state.mode = MOTOR_CURRENT;
		terminal_motor_state.param.current = current;
		return;
	}
	else if (!strcmp(up, "SPEED"))
	{
		if (argc < 3)
		{
			commands_printf("%s: -- smooth SPEED requires erpm argument -- For example: 'smooth SPEED 1000'",
							state_str(state));
			return;
		}
		float erpm;
		if (sscanf(argv[2], "%f", &erpm) == EOF)
		{
			commands_printf("%s: -- Can't parse '%s' as current value.",
							state_str(state), argv[2]);
			return;
		};
		terminal_command = SET_SMOOTH;
		terminal_motor_state.mode = MOTOR_SPEED;
		terminal_motor_state.param.erpm = erpm;
		return;
	}
	else
	{
		commands_printf("%s: -- 'smooth %s' not implemented",
						state_str(state), argv[1]);
	}
}
#endif