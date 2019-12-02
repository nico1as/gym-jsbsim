[![ Windows Build Status](http://badges.herokuapp.com/travis/galleon/gym-jsbsim?env=BADGE=windows&label=windows&branch=master)](https://travis-ci.org/galleon/gym-jsbsim) [![ Linux Build Status](http://badges.herokuapp.com/travis/galleon/gym-jsbsim?env=BADGE=linux&label=linux&branch=master)](https://travis-ci.org/galleon/gym-jsbsim) [![ MacOS Build Status](http://badges.herokuapp.com/travis/galleon/gym-jsbsim?env=BADGE=osx&label=OSX&branch=master)](https://travis-ci.org/galleon/gym-jsbsim) [![PyPI version](https://badge.fury.io/py/gym-jsbsim.svg)](https://badge.fury.io/py/gym-jsbsim) [![HitCount](http://hits.dwyl.com/galleon/gym-jsbsim.svg)](http://hits.dwyl.com/galleon/gym-jsbsim)



# JSBSim Aircraft Simulator for Open AI Gym Environment

## Are we able to learn an aircraft to fly (or taxi) ?

This project aims at creating realistic open AI GYM environements using the open source Flight Dynamic Model JSBSim.
Our work is initially based on the Gor-Ren repository (https://github.com/Gor-Ren/gym-jsbsim) on which we made several modifications:

 * Remove links to Flight-Gear to focus only on JSBSim and GYM environment making.
 * A complete link to JSBSim with an access to the full list of attributes.
 * A process to add extra features (like delta_heading for exemple) and the capability to call a dedicated function to update this attribute every time steps.
 * A feature to allow a ```set_state()``` and a ```get_state()``` to go back to a previous JSBSim state (very usefull if you want to use MCTS, ICT, IW and planning /scheduling approaches)
 * An autopilot for ground procedure (A320) based on 2 PIDs to manage throttles and brakes, to make the aircraft taxiing at a specific velocity.
 * A complete taxi environement with access to runaway path attributes (like distances and angles to next points) to train an aircraft to taxi autonomously.
 * Allow to define discrete GYM attributes.
 * ...


## Installation

### Mac OS / Linux
The package is directly installable with pip typing the following command :
```
pip install gym-jsbsim
```
### Windows
First, you have to install the Shapely library (issue [#39](https://github.com/galleon/gym-jsbsim/issues/39)) with :
* ##### Python 3.6 :
   ```
   pip install https://download.lfd.uci.edu/pythonlibs/t7epjj8p/Shapely-1.6.4.post2-cp36-cp36m-win_amd64.whl
   ```
* ##### Python 3.7 :
   ```
   pip install https://download.lfd.uci.edu/pythonlibs/t7epjj8p/Shapely-1.6.4.post2-cp37-cp37m-win_amd64.whl
   ```
Then, you can install the package with the pip install command :
```
pip install gym-jsbsim
```

## Environments

So far, is available:

 * [x] Heading Task: *GymJsbsim-HeadingControlTask-v0*: The aircraft should maintain its initial heading and altitude. During the simulation, the aircraft should turn to reach a new heading every 150 seconds with an incremental difficulty.
 * [x] Taxi task: *GymJsbsim-TaxiControlTask-v0*: The aircraft should follow a predifined trajectory on the runaway.
 * [x] Taxi with AutoPilot task: *GymJsbsim-TaxiapControlTask-v0*: The aircraft should follow a predifined trajectory on the runaway. The aircraft velocity is manage by an autopilot.
 * [x] Approach Task: *GymJsbsim-ApproachControlTask-v0*: The aircraft should decrese its altitude (it is still a draft environement and not realitic in regards to appraoch procedure).

## Heading Task

```
import gym
import gym_jsbsim

env = gym.make("GymJsbsim-HeadingControlTask-v0")
env.reset()
done = False

while not done:
   action = env.action_space.sample()
   state, reward, done, _ = env.step(action)
```

In this task, the aircraft should perform a stable steady flight following its initial heading and altitude. Every 150 seconds, a new target heading is set. At each time the target heading is more and more complicated to reach, starting with a delta of ±10° (random sign), following with ±20°, then ±30° and so on.

A terminal state is reached:
 * If the target heading is not reached with an accuracy of 10° during the 150 seconds.
 * If the aircraft is more than 100 feet away from its target altitude when the heading target changes.

The input state set is a vector of 9 parameters:

```
state_var = [c.delta_altitude, # the delta altitude between the aircraft and target altitude
	     c.delta_heading, # the delta heading between the aircraft and target heading
         c.attitude_pitch_rad,
         c.attitude_roll_rad,
	     c.velocities_v_down_fps, # the vertical velocity of the aircraft
	     c.velocities_vc_fps, # the air velocity of the aircraft
	     c.velocities_p_rad_sec, # roll axis velocity
	     c.velocities_q_rad_sec, # pitch axis velocity
	     c.velocities_r_rad_sec] # yaw axis velocity
```

The action set is a vector of 4 parameters:

```
action_var = [c.fcs_aileron_cmd_norm,
	      c.fcs_elevator_cmd_norm,
	      c.fcs_rudder_cmd_norm,
	      c.fcs_throttle_cmd_norm]
```

The reward is computed as a function of:
 * heading deviation from target
 * altitude deviation from target
 * roll angle
 * speed
 * acceleration perceived by the pilot in the cockpit

## Heading and Altitude Task

```
import gym
import gym_jsbsim

env = gym.make("GymJsbsim-HeadingAltitudeControlTask-v0")
env.reset()
done = False

while not done:
   action = env.action_space.sample()
   state, reward, done, _ = env.step(action)
```

This is the same as the Heading Task, but the target altitude also changes every 150 seconds. The altitude changes are: first ±100 feet (random sign), then ±200 feet, ±300 feet, and so on, but modulo 5000 feet and with a minimum target altitude of 3000 feet.

## Taxi Task

```
import gym
import gym_jsbsim

env = gym.make("GymJsbsim-TaxiControlTask-v0")
env.reset()
done = False

while not done:
   action = env.action_space.sample()
   state, reward, done, _ = env.step(action)
```

In this environnement, the aircraft should behave on ground and following a specific path, trying to be closest as possible to the runaway centerline.

For this environement, we have extract a path from the AMDB files of blagnac airport:

<p align="center">
  <img width="600" height="400" src=https://github.com/galleon/gym-jsbsim/blob/master/gym_jsbsim/docs/l2f_taxi_path.png?raw=true>
</p>

and extracted from this path a list of geodesic coordinates that the aircraft should follow.

As state set for this environment, we compute every timestep, the distance (di) and angle (ai) to the next 4 path points according to location of the aircraft (ie: d1 to d4 and a1 to a4)

<p align="center">
  <img src=https://github.com/galleon/gym-jsbsim/blob/master/gym_jsbsim/docs/l2f_taxistate.png?raw=true>
</p>


The full state set is:

```
state_var = [c.velocities_vc_fps,
	     c.shortest_dist,
	     c.d1, c.d2, c.d3, c.d4,
	     c.a1, c.a2, c.a3, c.a4]
```

The action set is define with 3 parameters that correspond to throttle, brake and steer commands:

```
action_var = [c.fcs_steer_cmd_norm,
	      c.fcs_center_brake_cmd_norm,
	      c.fcs_throttle_cmd_norm]
```

the reward is simply computed with the distance to the centerline

The scenario is over if the aircraft overtake the centerline from 10 meters or more.

## Taxi Auto Pilot Task

```
import gym
import gym_jsbsim

env = gym.make("GymJsbsim-TaxiapControlTask-v0")
env.reset()
done = False

while not done:
   action = env.action_space.sample()
   state, reward, done, _ = env.step(action)
```

This is the same Taxi environement with an autopilot to manage the aircraft velocity.
The velocity during straight line is set to 20 knots and the velocity during turn is set to 7 knots.

The state set is unchange:

```
state_var = [c.velocities_vc_fps,
	     c.shortest_dist,
	     c.d1, c.d2, c.d3, c.d4,
	     c.a1, c.a2, c.a3, c.a4]
```

but the action set is therefor focus on the steering command only:

```
action_var = [c.fcs_steer_cmd_norm]
```

the reward is unchanged and computed with the distance to the centerline


## Test

You could run a random agent with
```
python test_agent.py
```

## Results


## Links

Open AI GYM: https://github.com/openai/gym

JSBSim: https://github.com/JSBSim-Team/jsbsim

Gor-Ren Gym-Jsbsim repo: https://github.com/Gor-Ren/gym-jsbsim
