import gym
import gym.spaces
import numpy as np
import types
import math
import enum
import warnings
from collections import namedtuple
import gym_jsbsim.properties as prp
from gym_jsbsim import utils
from gym_jsbsim.simulation import Simulation
from gym_jsbsim.properties import BoundedProperty, Property
from gym_jsbsim.aircraft import Aircraft
from abc import ABC, abstractmethod
from typing import Optional, Sequence, Dict, Tuple, NamedTuple, Type
from .utils import reduce_reflex_angle_deg


class BaseFlightTask(ABC):
    """
    Abstract superclass for flight tasks, modules implementing specific environments in JSBSim.

    A task defines its own state space, action space, termination conditions and agent_reward function.

    Concrete subclasses should implement the following:
        state_variables attribute: tuple of Propertys, the task's state representation
        action_variables attribute: tuple of Propertys, the task's actions
        get_initial_conditions(): returns dict mapping InitialPropertys to initial values
        _is_terminal(): determines episode termination
        (optional) _new_episode_init(): performs any control input/initialisation on episode reset
        (optional) _update_custom_properties: updates any custom properties in the sim
    """

    state_variables: Tuple[BoundedProperty, ...]
    action_variables: Tuple[BoundedProperty, ...]
    State: Type[NamedTuple]
    Action: Type[NamedTuple]

    def __init__(self, debug: bool = False) -> None:
        self.last_state = None
        self._make_state_class()
        self._make_action_class()
        self.all_props = self._get_all_props()
        self.debug = debug

    def task_step(self, sim: Simulation, action: Sequence[float], sim_steps: int) \
            -> Tuple[np.ndarray, float, bool, Dict]:
        """
        Calculates new state, reward and termination.

        :param sim: a Simulation, the simulation from which to extract state
        :param action: sequence of floats, the agent's last action
        :param sim_steps: number of JSBSim integration steps to perform following action
            prior to making observation
        :return: tuple of (observation, reward, done, info) where,
            observation: array, agent's observation of the environment state
            reward: float, the reward for that step
            done: bool, True if the episode is over else False
            info: dict, optional, containing diagnostic info for debugging etc.
        """
        # input actions
        for prop, command in zip(self.action_variables, action):
            ### bounded the action
            if command <= prop.min:
                command = prop.min
            if command >= prop.max:
                command = prop.max
            sim[prop] = command
        
        # set throttle1 in case of more than 1 engine (ie: A320)
        try:
            sim[prp.all_engine_running] = -1
            sim[prp.throttle_1_cmd] = sim[prp.throttle_cmd]
        except KeyError:
            pass  # must be single-control aircraft     

        #print(f'past heading = {sim[prp.heading_deg]}, target = {sim[prp.target_heading_deg]}, past delta heading = {sim[prp.delta_heading]}')

        # run simulation
        for _ in range(sim_steps):
            sim.run()


        self._update_custom_properties(sim)

        # update delta heading and altitude according to the new heading and altitude
        sim[prp.delta_heading] = reduce_reflex_angle_deg(sim[prp.heading_deg] - sim[prp.target_heading_deg]) 
        sim[prp.delta_altitude] = sim[prp.altitude_sl_ft] - sim[prp.target_altitude_ft]
        #print(f'new heading = {sim[prp.heading_deg]}, target = {sim[prp.target_heading_deg]}, new delta heading = {state.position_delta_heading_to_target_deg} (from sim: {sim[prp.delta_heading]}')

        state = self.State(*(sim[prop] for prop in self.state_variables))
        action = self.Action(*(sim[prop] for prop in self.action_variables))
        done = self._is_terminal(sim, state)
        reward = self._get_reward(sim, self.last_state, action, state)
        if self.debug:
            self._validate_state(state, done, action, reward)
        self.last_state = state
        info = {'reward': reward}

        
        return state, reward, done, info

    def observe_first_state(self, sim: Simulation) -> np.ndarray:
        """
        Initialise any state/controls and get first state observation from reset sim.

        :param sim: Simulation, the environment simulation
        :return: np array, the first state observation of the episode
        """
        self._new_episode_init(sim)
        self._update_custom_properties(sim)
        state = self.State(*(sim[prop] for prop in self.state_variables))
        self.last_state = state
        return state
    
    def _new_episode_init(self, sim: Simulation) -> None:
        """
        This method is called at the start of every episode. It is used to set
        the value of any controls or environment properties not already defined
        in the task's initial conditions.

        By default it simply starts the aircraft engines.
        """
        sim.start_engines()
        sim.raise_landing_gear()

    def get_state_space(self) -> gym.Space:
        """ Get the task's state Space object """
        state_lows = np.array([state_var.min for state_var in self.state_variables])
        state_highs = np.array([state_var.max for state_var in self.state_variables])
        return gym.spaces.Box(low=state_lows, high=state_highs, dtype='float')

    def get_action_space(self) -> gym.Space:
        """ Get the task's action Space object """
        action_lows = np.array([act_var.min for act_var in self.action_variables])
        action_highs = np.array([act_var.max for act_var in self.action_variables])
        return gym.spaces.Box(low=action_lows, high=action_highs, dtype='float')

    def _make_state_class(self) -> None:
        """ Creates a namedtuple for readable State data """
        # get list of state property names, containing legal chars only
        legal_attribute_names = [prop.get_legal_name() for prop in
                                 self.state_variables]
        self.State = namedtuple('State', legal_attribute_names)
    
    def _make_action_class(self) -> None:
        """ Creates a namedtuple for readable Action data """
        # get list of action property names, containing legal chars only
        legal_attribute_names = [prop.get_legal_name() for prop in
                                 self.action_variables]
        self.Action = namedtuple('Action', legal_attribute_names)

    def _validate_state(self, state, done, action, reward):
        if any(math.isnan(el) for el in state):  # float('nan') in state doesn't work!
            msg = (f'Invalid state encountered!\n'
                   f'State: {state}\n'
                   f'Prev. State: {self.last_state}\n'
                   f'Action: {action}\n'
                   f'Terminal: {done}\n'
                   f'Reward: {reward}')
            warnings.warn(msg, RuntimeWarning)

    def _update_custom_properties(self, sim: Simulation) -> None:
        """ Calculates any custom properties which change every timestep. """
        pass

    def _get_all_props(self) -> Tuple:
        """
        Returns all the properties.

        :return: list of properties
        """
        props = []
        for name, item in prp.__dict__.items():
            if isinstance(item, (Property, BoundedProperty)):
                props.append(item)
        return props

    @abstractmethod
    def get_initial_conditions(self) -> Optional[Dict[Property, float]]:
        """
        Returns dictionary mapping initial episode conditions to values.

        Episode initial conditions (ICs) are defined by specifying values for
        JSBSim properties, represented by their name (string) in JSBSim.

        JSBSim uses a distinct set of properties for ICs, beginning with 'ic/'
        which differ from property names during the simulation, e.g. "ic/u-fps"
        instead of "velocities/u-fps". See https://jsbsim-team.github.io/jsbsim/

        :return: dict mapping string for each initial condition property to
            initial value, a float, or None to use Env defaults
        """
        ...

    @abstractmethod
    def _is_terminal(self, sim: Simulation, state: NamedTuple) -> bool:
        """ Determines whether the current episode should terminate.

        :param sim: the current simulation
        :param state: the current state
        :return: True if the episode should terminate else False
        """
        ...
    
    @abstractmethod
    def _get_reward(self, sim: Simulation, last_state: NamedTuple, action: NamedTuple, new_state: NamedTuple) -> float:
        """
        Returns the reward of transiting from the last state to the new state when executing a given action

        :param sim: the current simulation
        :param last_state: last state generated by the simulation
        :param action: action to execute in the last state
        :param new_state: new state generated by the simulation when executing the given action in the last state
        :return: Reward (float) of the transition generated by the simulation
        """
        ...
    
    @abstractmethod
    def get_props_to_output(self, sim: Simulation) -> Tuple:
        """
        Returns important properties of the task (can be embedded in the state or in the simulation in general)

        :param sim: the current simulation
        :return: list of properties
        """
        ...
