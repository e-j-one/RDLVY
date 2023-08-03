import logging

from collections import defaultdict, OrderedDict
import gym
from gym import spaces

from rware.utils import MultiAgentActionSpace, MultiAgentObservationSpace
from rware.utils.rdlvy_utils import select_highway_positions

from enum import Enum
import numpy as np

from typing import List, Tuple, Optional, Dict

import networkx as nx

import random

_AXIS_Z = 0
_AXIS_Y = 1
_AXIS_X = 2

_COLLISION_LAYERS = 2

_LAYER_AGENTS = 0
_LAYER_SHELFS = 1


class _VectorWriter:
    def __init__(self, size: int):
        self.vector = np.zeros(size, dtype=np.float32)
        self.idx = 0

    def write(self, data):
        data_size = len(data)
        self.vector[self.idx : self.idx + data_size] = data
        self.idx += data_size

    def skip(self, bits):
        self.idx += bits


class Action(Enum):
    NOOP = 0
    FORWARD = 1
    LEFT = 2
    RIGHT = 3
    TOGGLE_LOAD = 4

class Direction(Enum):
    UP = 0
    DOWN = 1
    LEFT = 2
    RIGHT = 3

class HighwayDirection(Enum):
    UP = 0
    DOWN = 1
    LEFT = 2
    RIGHT = 3
    ALL = 4
    NULL = 5

class RewardType(Enum):
    GLOBAL = 0
    INDIVIDUAL = 1
    TWO_STAGE = 2


class ObserationType(Enum):
    DICT = 0
    FLATTENED = 1
    IMAGE = 2

class ImageLayer(Enum):
    """
    Input layers of image-style observations
    """
    SHELVES = 0 # binary layer indicating shelves (also indicates carried shelves)
    REQUESTS = 1 # binary layer indicating requested shelves
    AGENTS = 2 # binary layer indicating agents in the environment (no way to distinguish agents)
    AGENT_DIRECTION = 3 # layer indicating agent directions as int (see Direction enum + 1 for values)
    AGENT_LOAD = 4 # binary layer indicating agents with load
    GOALS = 5 # binary layer indicating goal/ delivery locations
    ACCESSIBLE = 6 # binary layer indicating accessible cells (all but occupied cells/ out of map)


class Entity:
    def __init__(self, id_: int, x: int, y: int):
        self.id = id_
        self.prev_x = None
        self.prev_y = None
        self.x = x
        self.y = y


class Package(Entity):
    counter = 0

    def __init__(self, x, y):
        Package.counter += 1
        super().__init__(Package.counter, x, y)

    def move(self, x, y):
        self.x = x
        self.y = y

class Container:
    def __init__(self):
        self._num_packages = 0
        self.packages: List[Package] = []
    
    def load(self, package: Package):
        self.packages.append(package)
        self._num_packages += 1
    
    def unload(self):
        if self._num_packages > 0:
            self._num_packages -= 1
            return self.packages.pop(0) # FIFO # TODO: pop package w/ specific idx
        else:
            return None
    
    def count_packages(self):
        return self._num_packages
    
    def carrying_packages(self):
        if self._num_packages >0:
            return True
        else:
            return None
    
    def move_package(self, x, y):
        for package in self.packages:
            package.move(x,y)

class Agent(Entity):
    counter = 0

    def __init__(self, x: int, y: int, dir_: Direction, msg_bits: int):
        Agent.counter += 1
        super().__init__(Agent.counter, x, y)
        self.dir = dir_
        self.message = np.zeros(msg_bits)
        self.req_action: Optional[Action] = None
        # self.carrying_shelf: Optional[Shelf] = None
        self.canceled_action = None
        self.has_delivered = False

        self.container = Container()

    @property
    def collision_layers(self):
        if self.loaded:
            return (_LAYER_AGENTS, _LAYER_SHELFS)
        else:
            return (_LAYER_AGENTS,)
        
    def carrying_package(self):
        # return None if not carrying package
        # return True of carrying package else
        return self.container.carrying_packages()

    def req_location(self, grid_size) -> Tuple[int, int]:
        if self.req_action != Action.FORWARD:
            return self.x, self.y
        elif self.dir == Direction.UP:
            return self.x, max(0, self.y - 1)
        elif self.dir == Direction.DOWN:
            return self.x, min(grid_size[0] - 1, self.y + 1)
        elif self.dir == Direction.LEFT:
            return max(0, self.x - 1), self.y
        elif self.dir == Direction.RIGHT:
            return min(grid_size[1] - 1, self.x + 1), self.y

        raise ValueError(
            f"Direction is {self.dir}. Should be one of {[v for v in Direction]}"
        )

    def req_direction(self) -> Direction:
        wraplist = [Direction.UP, Direction.RIGHT, Direction.DOWN, Direction.LEFT]
        if self.req_action == Action.RIGHT:
            return wraplist[(wraplist.index(self.dir) + 1) % len(wraplist)]
        elif self.req_action == Action.LEFT:
            return wraplist[(wraplist.index(self.dir) - 1) % len(wraplist)]
        else:
            return self.dir
    
    def move_container(self):
        # drag container to agent's location
        self.container.move_package(self.x, self.y)


class Shelf(Entity):
    counter = 0

    def __init__(self, x, y):
        Shelf.counter += 1
        super().__init__(Shelf.counter, x, y)

    @property
    def collision_layers(self):
        return (_LAYER_SHELFS,)

class Warehouse(gym.Env):

    metadata = {"render.modes": ["human", "rgb_array"]}

    def __init__(
        self,
        n_agents: int,
        msg_bits: int,
        sensor_range: int,
        request_queue_size: int,
        max_inactivity_steps: Optional[int],
        max_steps: Optional[int],
        reward_type: RewardType,
        layout: str = None,
        observation_type: ObserationType=ObserationType.FLATTENED,
        use_full_obs:bool = False,
        image_observation_layers: List[ImageLayer]=[
            ImageLayer.SHELVES,
            ImageLayer.REQUESTS,
            ImageLayer.AGENTS,
            ImageLayer.AGENT_DIRECTION,
            ImageLayer.AGENT_LOAD,
            ImageLayer.GOALS,
            ImageLayer.ACCESSIBLE
        ],
        image_observation_directional: bool=True,
        normalised_coordinates: bool=False,
        package_carrying_capacity_per_agent: int = 3,
        block_size="small",
        shelf_columns: int = 0,
        column_height: int = 0,
        shelf_rows: int = 0,
    ):
        """The robotic warehouse environment

        Creates a grid world where multiple agents (robots)
        are supposed to collect shelfs, bring them to a goal
        and then return them.
        .. note:
            The grid looks like this:

            shelf
            columns
                vv
            ----------
            -XX-XX-XX-        ^
            -XX-XX-XX-  Column Height
            -XX-XX-XX-        v
            ----------
            -XX----XX-   <\
            -XX----XX-   <- Shelf Rows
            -XX----XX-   </
            ----------
            ----GG----

            G: is the goal positions where agents are rewarded if
            they bring the correct shelfs.

            The final grid size will be
            height: (column_height + 1) * shelf_rows + 2
            width: (2 + 1) * shelf_columns + 1

            The bottom-middle column will be removed to allow for
            robot queuing next to the goal locations

        :param shelf_columns: Number of columns in the warehouse
        :type shelf_columns: int
        :param column_height: Column height in the warehouse
        :type column_height: int
        :param shelf_rows: Number of columns in the warehouse
        :type shelf_rows: int
        :param n_agents: Number of spawned and controlled agents
        :type n_agents: int
        :param msg_bits: Number of communication bits for each agent
        :type msg_bits: int
        :param sensor_range: Range of each agents observation
        :type sensor_range: int
        :param request_queue_size: How many shelfs->Packages are simultaneously requested
        :type request_queue_size: int
        :param max_inactivity: Number of steps without a delivered shelf until environment finishes
        :type max_inactivity: Optional[int]
        :param reward_type: Specifies if agents are rewarded individually or globally
        :type reward_type: RewardType
        :param layout: A string for a custom warehouse layout. X are shelve locations, dots are corridors, and g are the goal locations. Ignores shelf_columns, shelf_height and shelf_rows when used.
        :type layout: str
        :param observation_type: Specifies type of observations
        :param image_observation_layers: Specifies types of layers observed if image-observations
            are used
        :type image_observation_layers: List[ImageLayer]
        :param image_observation_directional: Specifies whether image observations should be
            rotated to be directional (agent perspective) if image-observations are used
        :type image_observation_directional: bool
        :param normalised_coordinates: Specifies whether absolute coordinates should be normalised
            with respect to total warehouse size
        :type normalised_coordinates: bool
        """

        # self.goals: List[Tuple[int, int]] = []

        # requested_goals are as same size as request_queue_size
        # as package is delivered to the goal, it disappears and new goal appears
        # goal is randomly generated among goal_candidates and can not overlap
        self.goal_candidates: List[Tuple[int, int]] = []
        self.requested_goal_size = request_queue_size
        self.requested_goals: List[Tuple[int, int]] = []

        self.start_candidates: List[Tuple[int, int]] = []
        self.requested_starts: List[Tuple[int, int]] = []
        # self.starts_with_package_grid = np.zeros(self.grid_size, dtype=np.int32)

        if not layout:
            self._make_layout_from_params(shelf_columns, shelf_rows, column_height)
        else:
            self._make_layout_from_str(layout)

        self.n_agents = n_agents
        self.package_carrying_capacity_per_agent = package_carrying_capacity_per_agent
        self.msg_bits = msg_bits
        self.sensor_range = sensor_range
        self.max_inactivity_steps: Optional[int] = max_inactivity_steps
        self.reward_type = reward_type
        self.reward_range = (0, 1)
        self.block_size = block_size

        self._cur_inactive_steps = None
        self._cur_steps = 0
        self.max_steps = max_steps
        
        self.normalised_coordinates = normalised_coordinates

        sa_action_space = [len(Action), *msg_bits * (2,)]
        if len(sa_action_space) == 1:
            sa_action_space = spaces.Discrete(sa_action_space[0])
        else:
            sa_action_space = spaces.MultiDiscrete(sa_action_space)
        self.action_space = spaces.Tuple(tuple(n_agents * [sa_action_space]))

        self.request_queue_size = request_queue_size
        self.request_queue = []

        self.agents: List[Agent] = []

        # default values:
        self.use_full_obs = use_full_obs
        self.fast_obs = None
        self.image_obs = None
        self.observation_space = None
        if observation_type == ObserationType.IMAGE:
            self._use_image_obs(image_observation_layers, image_observation_directional)
        else:
            # used for DICT observation type and needed as preceeding stype to generate
            # FLATTENED observations as well
            self._use_slow_obs()

        # for performance reasons we
        # can flatten the obs vector
        if observation_type == ObserationType.FLATTENED:
            self._use_fast_obs()

        self.renderer = None

    def _make_layout_from_params(self, shelf_columns, shelf_rows, column_height):
        # TODO: deprecate layer from param
        assert shelf_columns % 2 == 1, "Only odd number of shelf columns is supported"

        self.grid_size = (
            (column_height + 1) * shelf_rows + 2,
            (2 + 1) * shelf_columns + 1,
        )
        self.column_height = column_height
        self.grid = np.zeros((_COLLISION_LAYERS, *self.grid_size), dtype=np.int32)
        self.goals = [
            (self.grid_size[1] // 2 - 1, self.grid_size[0] - 1),
            (self.grid_size[1] // 2, self.grid_size[0] - 1),
        ]

        self.highways = np.zeros(self.grid_size, dtype=np.int32)

        highway_func = lambda x, y: (
            (x % 3 == 0)  # vertical highways
            or (y % (self.column_height + 1) == 0)  # horizontal highways
            or (y == self.grid_size[0] - 1)  # delivery row
            or (  # remove a box for queuing
                (y > self.grid_size[0] - (self.column_height + 3))
                and ((x == self.grid_size[1] // 2 - 1) or (x == self.grid_size[1] // 2))
            )
        )
        for x in range(self.grid_size[1]):
            for y in range(self.grid_size[0]):
                self.highways[y, x] = highway_func(x, y)

    def _make_layout_from_str(self, layout):
        layout = layout.strip()
        layout = layout.replace(" ", "")
        grid_height = layout.count("\n") + 1
        lines = layout.split("\n")
        grid_width = len(lines[0])
        for line in lines:
            assert len(line) == grid_width, "Layout must be rectangular"

        self.grid_size = (grid_height, grid_width)
        self.grid = np.zeros((_COLLISION_LAYERS, *self.grid_size), dtype=np.int32)
        self.requested_package_grid = np.zeros(self.grid_size, dtype=np.int32)
        self.requested_goal_grid = np.zeros(self.grid_size, dtype=np.int32)
        self.highways = np.zeros(self.grid_size, dtype=np.int32)
        self.highways_info = np.array([HighwayDirection.NULL for _ in range(self.grid_size[0]*self.grid_size[1])]).reshape(self.grid_size)

        # start point where packages are waiting
        # cell indicates number of packages waiting
        self.starts_with_package_grid = np.zeros(self.grid_size, dtype=np.int32)

        for y, line in enumerate(lines):
            for x, char in enumerate(line):
                assert char.lower() in "gxsrldu."
                if char.lower() == "g":
                    # self.goals.append((x, y))
                    self.goal_candidates.append((x, y))
                    self.highways[y, x] = 1
                    # self.highways_info[y, x] = HighwayDirection.ALL
                elif char.lower() == ".":
                    self.highways[y, x] = 1
                    self.highways_info[y, x] = HighwayDirection.ALL
                elif char.lower() == "s":
                    self.start_candidates.append((x,y))
                    self.highways[y, x] = 1
                    # self.highways_info[y, x] = HighwayDirection.ALL
                elif char.lower() == "r":
                    self.highways[y, x] = 1
                    self.highways_info[y, x] = HighwayDirection.RIGHT
                elif char.lower() == "l":
                    self.highways[y, x] = 1
                    self.highways_info[y, x] = HighwayDirection.LEFT
                elif char.lower() == "d":
                    self.highways[y, x] = 1
                    self.highways_info[y, x] = HighwayDirection.DOWN
                elif char.lower() == "u":
                    self.highways[y, x] = 1
                    self.highways_info[y, x] = HighwayDirection.UP

        assert len(self.goal_candidates) >= 1, "At least one goal is required"

    def _use_image_obs(self, image_observation_layers, directional=True):
        """
        Set image observation space
        :param image_observation_layers (List[ImageLayer]): list of layers to use as image channels
        :param directional (bool): flag whether observations should be directional (pointing in
            direction of agent or north-wise)
        """
        self.image_obs = True
        self.fast_obs = False
        self.image_observation_directional = directional
        self.image_observation_layers = image_observation_layers

        observation_shape = (1 + 2 * self.sensor_range, 1 + 2 * self.sensor_range)

        layers_min = []
        layers_max = []
        for layer in image_observation_layers:
            if layer == ImageLayer.AGENT_DIRECTION:
                # directions as int
                layer_min = np.zeros(observation_shape, dtype=np.float32)
                layer_max = np.ones(observation_shape, dtype=np.float32) * max([d.value + 1 for d in Direction])
            else:
                # binary layer
                layer_min = np.zeros(observation_shape, dtype=np.float32)
                layer_max = np.ones(observation_shape, dtype=np.float32)
            layers_min.append(layer_min)
            layers_max.append(layer_max)

        # total observation
        min_obs = np.stack(layers_min)
        max_obs = np.stack(layers_max)
        self.observation_space = spaces.Tuple(
            tuple([spaces.Box(min_obs, max_obs, dtype=np.float32)] * self.n_agents)
        )

    def _use_slow_obs(self):
        self.fast_obs = False

        self._obs_bits_for_self = 4 + len(Direction)
        self._obs_bits_per_agent = 1 + len(Direction) + self.msg_bits
        self._obs_bits_per_shelf = 2
        self._obs_bits_for_requests = 2

        self._obs_sensor_locations = (1 + 2 * self.sensor_range) ** 2

        self._obs_length = (
            self._obs_bits_for_self
            + self._obs_sensor_locations * self._obs_bits_per_agent
            + self._obs_sensor_locations * self._obs_bits_per_shelf
        )

        if self.normalised_coordinates:
            location_space = spaces.Box(
                    low=0.0,
                    high=1.0,
                    shape=(2,),
                    dtype=np.float32,
            )
        else:
            location_space = spaces.MultiDiscrete(
                [self.grid_size[1], self.grid_size[0]]
            )

        self.observation_space = spaces.Tuple(
            tuple(
                [
                    spaces.Dict(
                        OrderedDict(
                            {
                                "self": spaces.Dict(
                                    OrderedDict(
                                        {
                                            "location": location_space,
                                            "carrying_shelf": spaces.MultiDiscrete([2]), # Now works as carrying_package # TODO: change name
                                            "direction": spaces.Discrete(4),
                                            "on_highway": spaces.MultiBinary(1), # Should always be true
                                        }
                                    )
                                ),
                                "sensors": spaces.Tuple(
                                    self._obs_sensor_locations
                                    * (
                                        spaces.Dict(
                                            OrderedDict(
                                                {
                                                    "has_agent": spaces.MultiBinary(1),
                                                    "direction": spaces.Discrete(4),
                                                    "local_message": spaces.MultiBinary(
                                                        self.msg_bits
                                                    ),
                                                    "has_shelf": spaces.MultiBinary(1), # Leave it as location of shelf # TODO: change name
                                                    # "shelf_requested": spaces.MultiBinary(1), # Now works as package_packages # TODO: change name
                                                    "shelf_requested": spaces.Discrete(1), # Now works as package_packages # TODO: change name
                                                    "goal_requested": spaces.Discrete(1),
                                                }
                                            )
                                        ),
                                    )
                                ),
                            }
                        )
                    )
                    for _ in range(self.n_agents)
                ]
            )
        )

    def _use_fast_obs(self):
        if self.fast_obs:
            return

        self.fast_obs = True
        ma_spaces = []
        for sa_obs in self.observation_space:
            flatdim = spaces.flatdim(sa_obs)
            ma_spaces += [
                spaces.Box(
                    low=-float("inf"),
                    high=float("inf"),
                    shape=(flatdim,),
                    dtype=np.float32,
                )
            ]

        self.observation_space = spaces.Tuple(tuple(ma_spaces))

    def _is_highway(self, x: int, y: int) -> bool:
        return self.highways[y, x]

    def _make_obs(self, agent):
        if self.image_obs:
            # write image observations
            if agent.id == 1:
                layers = []
                org_layers = []
                # first agent's observation --> update global observation layers
                for layer_type in self.image_observation_layers:
                    if layer_type == ImageLayer.SHELVES:
                        layer = self.grid[_LAYER_SHELFS].copy().astype(np.float32)
                        # set all occupied shelf cells to 1.0 (instead of shelf ID)
                        layer[layer > 0.0] = 1.0
                        # print("SHELVES LAYER")
                    elif layer_type == ImageLayer.REQUESTS:
                        layer = np.zeros(self.grid_size, dtype=np.float32)
                        # for requested_shelf in self.request_queue:
                            # layer[requested_shelf.y, requested_shelf.x] = 1.0
                        for package in self.request_queue:
                            layer[package.y, package.x] = 1.0
                        # print("REQUESTS LAYER")
                    elif layer_type == ImageLayer.AGENTS:
                        layer = self.grid[_LAYER_AGENTS].copy().astype(np.float32)
                        # set all occupied agent cells to 1.0 (instead of agent ID)
                        layer[layer > 0.0] = 1.0
                        # print("AGENTS LAYER")
                    elif layer_type == ImageLayer.AGENT_DIRECTION:
                        layer = np.zeros(self.grid_size, dtype=np.float32)
                        for ag in self.agents:
                            agent_direction = ag.dir.value + 1
                            layer[ag.y, ag.x] = float(agent_direction)
                        # print("AGENT DIRECTIONS LAYER")
                    elif layer_type == ImageLayer.AGENT_LOAD:
                        layer = np.zeros(self.grid_size, dtype=np.float32)
                        for ag in self.agents:
                            # if ag.carrying_shelf is not None:
                            #     layer[ag.x, ag.y] = 1.0
                            if ag.carrying_package() is not None:
                                layer[ag.y, ag.x] = 1.0
                        # print("AGENT LOAD LAYER")
                    elif layer_type == ImageLayer.GOALS:
                        layer = np.zeros(self.grid_size, dtype=np.float32)
                        # for goal_y, goal_x in self.goals:
                        #     layer[goal_x, goal_y] = 1.0
                        for goal_y, goal_x in self.requested_goals:
                            layer[goal_x, goal_y] = 1.0
                        # print("GOALS LAYER")
                    elif layer_type == ImageLayer.ACCESSIBLE:
                        layer = (1. - self.grid[_LAYER_SHELFS].copy().astype(np.float32) > 0)
                        # layer = np.ones(self.grid_size, dtype=np.float32)
                        # for ag in self.agents:
                            # layer[ag.y, ag.x] = 0.0 # TODO: change accessiblity as road become two-way

                    # pad with 0s for out-of-map cells
                    org_layer = layer
                    org_layers.append(org_layer)
                    layer = np.pad(layer, self.sensor_range, mode="constant")
                    layers.append(layer)
                self.global_layers = np.stack(layers)
                self.org_global_layers = np.stack(org_layers)
            if self.use_full_obs:
                return self.org_global_layers

            # global information was generated --> get information for agent
            start_x = agent.y
            end_x = agent.y + 2 * self.sensor_range + 1
            start_y = agent.x
            end_y = agent.x + 2 * self.sensor_range + 1
            obs = self.global_layers[:, start_x:end_x, start_y:end_y]

            if self.image_observation_directional:
                # rotate image to be in direction of agent
                if agent.dir == Direction.DOWN:
                    # rotate by 180 degrees (clockwise)
                    obs = np.rot90(obs, k=2, axes=(1,2))
                elif agent.dir == Direction.LEFT:
                    # rotate by 90 degrees (clockwise)
                    obs = np.rot90(obs, k=3, axes=(1,2))
                elif agent.dir == Direction.RIGHT:
                    # rotate by 270 degrees (clockwise)
                    obs = np.rot90(obs, k=1, axes=(1,2))
                # no rotation needed for UP direction
            return obs

        min_x = agent.x - self.sensor_range
        max_x = agent.x + self.sensor_range + 1

        min_y = agent.y - self.sensor_range
        max_y = agent.y + self.sensor_range + 1

        # sensors
        if (
            (min_x < 0)
            or (min_y < 0)
            or (max_x > self.grid_size[1])
            or (max_y > self.grid_size[0])
        ):
            padded_agents = np.pad(
                self.grid[_LAYER_AGENTS], self.sensor_range, mode="constant"
            )
            padded_shelfs = np.pad(
                self.grid[_LAYER_SHELFS], self.sensor_range, mode="constant"
            )
            padded_requested_packages = np.pad(
                self.requested_package_grid, self.sensor_range, mode="constant"
            )
            padded_requested_goals = np.pad(
                self.requested_goal_grid, self.sensor_range, mode="constant"
            )
            # + self.sensor_range due to padding
            min_x += self.sensor_range
            max_x += self.sensor_range
            min_y += self.sensor_range
            max_y += self.sensor_range

        else:
            padded_agents = self.grid[_LAYER_AGENTS]
            padded_shelfs = self.grid[_LAYER_SHELFS]
            padded_requested_packages = self.requested_package_grid
            padded_requested_goals = self.requested_goal_grid

        agents = padded_agents[min_y:max_y, min_x:max_x].reshape(-1)
        shelfs = padded_shelfs[min_y:max_y, min_x:max_x].reshape(-1)
        requested_packages = padded_requested_packages[min_y:max_y, min_x:max_x].reshape(-1)
        requested_goals = padded_requested_goals[min_y:max_y, min_x:max_x].reshape(-1)

        if self.fast_obs:
            # write flattened observations
            obs = _VectorWriter(self.observation_space[agent.id - 1].shape[0])

            if self.normalised_coordinates:
                agent_x = agent.x / (self.grid_size[1] - 1)
                agent_y = agent.y / (self.grid_size[0] - 1)
            else:
                agent_x = agent.x
                agent_y = agent.y

            # obs.write([agent_x, agent_y, int(agent.carrying_shelf is not None)])
            obs.write([agent_x, agent_y, int(agent.carrying_package() is not None)])
            direction = np.zeros(4)
            direction[agent.dir.value] = 1.0
            obs.write(direction)
            obs.write([int(self._is_highway(agent.x, agent.y))])

            for i, (id_agent, id_shelf, num_requested_packages, num_requested_goals) in enumerate(zip(agents, shelfs, requested_packages, requested_goals)):
                if id_agent == 0:
                    obs.skip(1)
                    obs.write([1.0])
                    obs.skip(3 + self.msg_bits)
                else:
                    obs.write([1.0])
                    direction = np.zeros(4)
                    direction[self.agents[id_agent - 1].dir.value] = 1.0
                    obs.write(direction)
                    if self.msg_bits > 0:
                        obs.write(self.agents[id_agent - 1].message)
                if id_shelf == 0:
                    obs.skip(1)
                else:
                    obs.write(
                        # [1.0, int(self.shelfs[id_shelf - 1] in self.request_queue)]
                        [1.0]
                    )
                if num_requested_packages == 0:
                    obs.skip(1)
                if num_requested_goals == 0:
                    obs.skip(1)
                else:
                    obs.write([1.0])

            return obs.vector
 
        # write dictionary observations
        obs = {}
        if self.normalised_coordinates:
            agent_x = agent.x / (self.grid_size[1] - 1)
            agent_y = agent.y / (self.grid_size[0] - 1)
        else:
            agent_x = agent.x
            agent_y = agent.y
        # --- self data
        obs["self"] = {
            "location": np.array([agent_x, agent_y]),
            # "carrying_shelf": [int(agent.carrying_shelf is not None)],
            "carrying_shelf": [int(agent.carrying_package() is not None)], 
            "direction": agent.dir.value,
            "on_highway": [int(self._is_highway(agent.x, agent.y))],
        }
        # --- sensor data
        obs["sensors"] = tuple({} for _ in range(self._obs_sensor_locations))

        # find neighboring agents
        for i, id_ in enumerate(agents):
            if id_ == 0:
                obs["sensors"][i]["has_agent"] = [0]
                obs["sensors"][i]["direction"] = 0
                obs["sensors"][i]["local_message"] = self.msg_bits * [0]
            else:
                obs["sensors"][i]["has_agent"] = [1]
                obs["sensors"][i]["direction"] = self.agents[id_ - 1].dir.value
                obs["sensors"][i]["local_message"] = self.agents[id_ - 1].message # TODO: change messages

        # find neighboring shelfs:
        for i, id_ in enumerate(shelfs):
            if id_ == 0:
                obs["sensors"][i]["has_shelf"] = [0]
                # obs["sensors"][i]["shelf_requested"] = [0]
            else:
                obs["sensors"][i]["has_shelf"] = [1]
                # obs["sensors"][i]["shelf_requested"] = [
                #     int(self.shelfs[id_ - 1] in self.request_queue)
                # ]

        # find neighboring start position with requested packages
        for i, num_ in enumerate(requested_packages):
            if num_ == 0:
                obs["sensors"][i]["shelf_requested"] = [0]
            else:
                obs["sensors"][i]["shelf_requested"] = [1]
        
        # find neighboring start position with requested goals
        for i, num_ in enumerate(requested_goals):
            if num_ == 0:
                obs["sensors"][i]["goal_requested"] = [0]
            else:
                obs["sensors"][i]["goal_requested"] = [1]
        
        return obs

    def _recalc_grid(self):
        self.grid[:] = 0
        for s in self.shelfs:
            self.grid[_LAYER_SHELFS, s.y, s.x] = s.id

        for a in self.agents:
            self.grid[_LAYER_AGENTS, a.y, a.x] = a.id

        self.requested_package_grid[:] = 0
        for p in self.request_queue:
            self.requested_package_grid[p.y, p.x] += 1

    def _create_packages(self):
        _new_package_orders = []
        for i in range(self.request_queue_size):
            _random_start_pos = random.choice(self.start_candidates)
            self.starts_with_package_grid[_random_start_pos[1], _random_start_pos[0]] += 1
            _package = Package(_random_start_pos[0], _random_start_pos[1])
            _new_package_orders.append(_package)
        return _new_package_orders
    
    def _create_new_package(self):
        _random_start_pos = random.choice(self.start_candidates)
        self.starts_with_package_grid[_random_start_pos[1], _random_start_pos[0]] += 1
        _package = Package(_random_start_pos[0], _random_start_pos[1])
        return _package

    def _print_highways_info(self):
        for i in range(self.grid_size[0]):
            line_info = []
            for j in range(self.grid_size[1]):
                line_info.append(self.highways_info[i, j].value)
            print(line_info)
            print()
    
    def reset(self):
        Shelf.counter = 0
        Agent.counter = 0
        self._cur_inactive_steps = 0
        self._cur_steps = 0
        # n_xshelf = (self.grid_size[1] - 1) // 3
        # n_yshelf = (self.grid_size[0] - 2) // 9

        # make the shelfs
        self.shelfs = [
            Shelf(x, y)
            for y, x in zip(
                np.indices(self.grid_size)[0].reshape(-1),
                np.indices(self.grid_size)[1].reshape(-1),
            )
            if not self._is_highway(x, y)
        ]

        # self.request_queue = list(
        #     np.random.choice(self.shelfs, size=self.request_queue_size, replace=False)
        # )
        self.request_queue = self._create_packages()
        self.requested_goals = random.sample(self.goal_candidates, self.requested_goal_size)
        _unselected_goals = [goal for goal in self.goal_candidates if goal not in self.requested_goals]
        for _u_goals in _unselected_goals:
            self.highways[_u_goals[1], _u_goals[0]] = 0

        # spawn agents at random locations
        # agent_locs = np.random.choice(
        #     np.arange(self.grid_size[0] * self.grid_size[1]),
        #     size=self.n_agents,
        #     replace=False,
        # )
        # agent_locs = np.unravel_index(agent_locs, self.grid_size)
        agent_locs = select_highway_positions(self.highways, self.n_agents)
        
        # and direction
        agent_dirs = np.random.choice([d for d in Direction], size=self.n_agents)

        self.agents = [
            Agent(x, y, dir_, self.msg_bits)
            for y, x, dir_ in zip(*agent_locs, agent_dirs)
        ]

        self._recalc_grid()
        
        # Highways info update in edge points
        # Agents can turn around in the edge of the map.
             
        if self.use_full_obs:
            return self._make_obs(self.agents[0])
        else:
            return tuple([self._make_obs(agent) for agent in self.agents])
        # for s in self.shelfs:
        #     self.grid[0, s.y, s.x] = 1
        # print(self.grid[0])

    def _is_start_pose_with_package(self, x, y):
        if self.requested_package_grid[y,x] > 0:
            return True
        else:
            return False

    def _load_package_from_stat_pose(self, x, y):
        for i, package in enumerate(self.request_queue):
            if package.x == x and package.y == y:
                del self.request_queue[i]
                self.starts_with_package_grid[y, x] -= 1
                return package
        raise ValueError(f'No package found at location ({x}, {y})')

    def _replace_goal(self, x, y):
        for i, goal_pose in enumerate(self.requested_goals):
            if goal_pose[0] == x and goal_pose[1] == y:
                del self.requested_goals[i]
                break
        
        _unselected_goals = [goal for goal in self.goal_candidates if goal not in self.requested_goals]
        _new_goal = random.choice(_unselected_goals)
        self.highways[_new_goal[1], _new_goal[0]] = 1
        self.requested_goals.append(_new_goal)

    def _check_nearby(self, pos):
        num_nearby_ALL, num_nearby_NULL, num_nearby_UP, num_nearby_DOWN, num_nearby_LEFT, num_nearby_RIGHT = 0, 0, 0, 0, 0, 0
        for i, j  in zip([0, 0, 1, -1], [1, -1 ,0, 0]):
            if (pos[1]+i > self.grid_size[0]-1 or pos[1]+i < 0):
                i = 0
            elif (pos[0]+j > self.grid_size[1]-1 or pos[0]+j < 0):
                j = 0
            # print(pos[1], i, pos[0], j)
            if self.highways_info[pos[1]+i, pos[0]+j] == HighwayDirection.ALL:
                num_nearby_ALL += 1
            elif self.highways_info[pos[1]+i, pos[0]+j] == HighwayDirection.NULL:
                num_nearby_NULL += 1
            elif self.highways_info[pos[1]+i, pos[0]+j] == HighwayDirection.UP:
                num_nearby_UP += 1
            elif self.highways_info[pos[1]+i, pos[0]+j] == HighwayDirection.DOWN:
                num_nearby_DOWN += 1
            elif self.highways_info[pos[1]+i, pos[0]+j] == HighwayDirection.LEFT:
                num_nearby_LEFT += 1
            elif self.highways_info[pos[1]+i, pos[0]+j] == HighwayDirection.RIGHT:
                num_nearby_RIGHT += 1
        
        assert num_nearby_ALL+num_nearby_NULL+num_nearby_UP+num_nearby_DOWN+num_nearby_LEFT+num_nearby_RIGHT == 4
        
        return num_nearby_ALL, num_nearby_NULL, num_nearby_UP, num_nearby_DOWN, num_nearby_LEFT, num_nearby_RIGHT

    # Intersection type 3:
    # ex) x x x x d u x x x x
    #     l l l l . u x x x x
    #     r r r r r(.)x x x x
    #     x x x x x x x x x x
    # the () point: outer corner point of corner street.
    #
    # Intersection type 1:
    # ex) x x x x d u x x x x
    #     l l l l(.)u x x x x
    #     r r r r r . x x x x
    #     x x x x x x x x x x
    # the () point: inner corner point of corner street.
    #
    # for intersection type 1...
    # cannot decide where to go with only cross information.
    # Need to find where the wall is.
    def _check_corner(self, pos):
        assert self.highways_info[pos[1], pos[0]] == HighwayDirection.ALL
        for i, j in zip([1, 1, -1, -1], [1, -1 ,1, -1]):
            if self.highways_info[pos[1]+i, pos[0]+j] == HighwayDirection.NULL:
                return (j, i)
        AssertionError("The position is not an intersection type 1.")

    # check if the req_dir is possiple in that point of highways_info  
    # ex) cannot see down or left on the highways_info==UP point
    def _is_possible_dir(self, highway_dir: HighwayDirection, req_dir:Direction, agent_pos: tuple) -> bool:
        if highway_dir == HighwayDirection.UP:
            if req_dir == Direction.DOWN or req_dir==Direction.LEFT:
                return False
        elif highway_dir == HighwayDirection.DOWN:
            if req_dir == Direction.UP or req_dir==Direction.RIGHT:
                return False
        elif highway_dir == HighwayDirection.LEFT:
            if req_dir == Direction.RIGHT or req_dir==Direction.DOWN:
                return False
        elif highway_dir == HighwayDirection.RIGHT:
            if req_dir == Direction.LEFT or req_dir==Direction.UP:
                return False
        # Intersection logic
        elif highway_dir == HighwayDirection.ALL: 
            n_ALL, n_NULL, n_UP, n_DOWN, n_LEFT, n_RIGHT = self._check_nearby(agent_pos)
            # Intersection type 3
            # when the number of possible direction is 3 (except the direction where the agent comes from)
            if n_NULL == 2:
                if n_UP * n_LEFT: 
                    if req_dir == Direction.DOWN: return False
                elif n_DOWN * n_RIGHT:
                    if req_dir == Direction.UP: return False
                elif n_LEFT * n_DOWN:
                    if req_dir == Direction.RIGHT: return False
                elif n_RIGHT * n_UP:
                    if req_dir == Direction.LEFT: return False
                else:
                    AssertionError("Somethings wrong in layout. Check the intersection_3 part.")
                return True 
            # Intersection type 1
            # when the number of possible direction is 1
            elif n_NULL == 0:
                if self._check_corner(agent_pos) == (1, 1):
                    # the corner is at the botton-right
                    if req_dir == Direction.RIGHT: return True
                elif self._check_corner(agent_pos) == (1, -1):
                    # up-right
                    if req_dir == Direction.UP: return True
                elif self._check_corner(agent_pos) == (-1, 1):
                    # bottom-left
                    if req_dir == Direction.DOWN: return True
                elif self._check_corner(agent_pos) == (-1, -1):
                    # up-left
                    if req_dir == Direction.LEFT: return True
                return False
            # u-turn points (the end of road at the the edge of the layout)
            elif n_NULL == 1:
                if agent_pos[1]+1 > self.grid_size[0] - 1:
                    if self.highways_info[agent_pos[1], agent_pos[0]-1] == HighwayDirection.NULL:
                        if (agent_pos[1], agent_pos[0]+1) in self.requested_goals:
                            if req_dir == Direction.RIGHT or req_dir == Direction.LEFT:
                                return True 
                        else:
                            if req_dir == Direction.RIGHT:
                                return True
                    elif self.highways_info[agent_pos[1], agent_pos[0]+1] == HighwayDirection.NULL:
                        if (agent_pos[1], agent_pos[0]-1) in self.requested_goals:
                            if req_dir == Direction.UP or req_dir == Direction.RIGHT:
                                return True 
                        else:
                            if req_dir == Direction.UP:
                                return True
                elif agent_pos[1]-1 < 0:
                    if self.highways_info[agent_pos[1], agent_pos[0]+1] == HighwayDirection.NULL:
                        if (agent_pos[1], agent_pos[0]+1) in self.requested_goals:
                            if req_dir == Direction.LEFT or req_dir == Direction.RIGHT:
                                return True 
                        else:
                            if req_dir == Direction.LEFT:
                                return True
                    elif self.highways_info[agent_pos[1], agent_pos[0]-1] == HighwayDirection.NULL:
                        if (agent_pos[1], agent_pos[0]-1) in self.requested_goals:
                            if req_dir == Direction.DOWN or req_dir == Direction.LEFT:
                                return True 
                        else:
                            if req_dir == Direction.DOWN:
                                return True
                elif agent_pos[0]+1 > self.grid_size[1] - 1:
                    if self.highways_info[agent_pos[1]+1, agent_pos[0]] == HighwayDirection.NULL:
                        if (agent_pos[1]+1, agent_pos[0]) in self.requested_goals:
                            if req_dir == Direction.UP or req_dir == Direction.DOWN:
                                return True 
                        else:
                            if req_dir == Direction.UP:
                                return True
                    elif self.highways_info[agent_pos[1]-1, agent_pos[0]] == HighwayDirection.NULL:
                        if (agent_pos[1]-1, agent_pos[0]) in self.requested_goals:
                            if req_dir == Direction.RIGHT or req_dir == Direction.UP:
                                return True 
                        else:
                            if req_dir == Direction.RIGHT:
                                return True
                elif agent_pos[0]-1 < 0:
                    if self.highways_info[agent_pos[1]-1, agent_pos[0]] == HighwayDirection.NULL:
                        if (agent_pos[1]+1, agent_pos[0]) in self.requested_goals:
                            if req_dir == Direction.DOWN or req_dir == Direction.UP:
                                return True 
                        else:
                            if req_dir == Direction.DOWN:
                                return True
                    elif self.highways_info[agent_pos[1]+1, agent_pos[0]] == HighwayDirection.NULL:
                        if (agent_pos[1]-1, agent_pos[0]) in self.requested_goals:
                            if req_dir == Direction.RIGHT or req_dir == Direction.DOWN:
                                return True 
                        else:
                            if req_dir == Direction.RIGHT:
                                return True
                return False
        return True 
    
    def step(
        self, actions: List[Action]
    ) -> Tuple[List[np.ndarray], List[float], List[bool], Dict]:
        assert len(actions) == len(self.agents)

        for agent, action in zip(self.agents, actions):
            if self.msg_bits > 0:
                agent.req_action = Action(action[0])
                agent.message[:] = action[1:]
            else:
                agent.req_action = Action(action)
        
        _unselected_goals = [goal for goal in self.goal_candidates if goal not in self.requested_goals]
        for _u_goals in _unselected_goals:
            if not self.grid[_LAYER_AGENTS, _u_goals[1], _u_goals[0]]:
                self.highways[_u_goals[1], _u_goals[0]] = 0

        # # stationary agents will certainly stay where they are
        # stationary_agents = [agent for agent in self.agents if agent.action != Action.FORWARD]

        # # forward agents will move only if they avoid collisions
        # forward_agents = [agent for agent in self.agents if agent.action == Action.FORWARD]
        commited_agents = set()

        G = nx.DiGraph()

        for agent in self.agents:
            start = agent.x, agent.y
            target = agent.req_location(self.grid_size)

            # shelf is deprecated
            # if (
            #     agent.carrying_shelf
            #     and start != target
            #     and self.grid[_LAYER_SHELFS, target[1], target[0]]
            #     and not (
            #         self.grid[_LAYER_AGENTS, target[1], target[0]]
            #         and self.agents[
            #             self.grid[_LAYER_AGENTS, target[1], target[0]] - 1
            #         ].carrying_shelf
            #     )
            # ):
            #     # there's a standing shelf at the target location
            #     # our agent is carrying a shelf so there's no way
            #     # this movement can succeed. Cancel it.
            #     agent.req_action = Action.NOOP
            #     G.add_edge(start, start)
            if (
                # check if target position is possible to go when action==Forward:
                agent.req_action == Action.FORWARD
                and (
                    not self._is_possible_dir(self.highways_info[agent.y, agent.x], agent.dir, start)
                )
            ):
                agent.req_action = Action.NOOP
                G.add_edge(start, start)
            elif (
                not self._is_highway(target[0], target[1])
            ):
                # start, goal candidiates are all on the highway
                # agent can't move outside of highway
                agent.req_action = Action.NOOP
                G.add_edge(start, start)
            else:
                G.add_edge(start, target)

        wcomps= [G.subgraph(c).copy() for c in nx.weakly_connected_components(G)]

        for comp in wcomps:
            try:
                # if we find a cycle in this component we have to
                # commit all nodes in that cycle, and nothing else
                cycle = nx.algorithms.find_cycle(comp)
                if len(cycle) == 2:
                    # we have a situation like this: [A] <-> [B]
                    # which is physically impossible. so skip
                    continue
                for edge in cycle:
                    start_node = edge[0]
                    agent_id = self.grid[_LAYER_AGENTS, start_node[1], start_node[0]]
                    if agent_id > 0:
                        commited_agents.add(agent_id)
            except nx.NetworkXNoCycle:

                longest_path = nx.algorithms.dag_longest_path(comp)
                for x, y in longest_path:
                    agent_id = self.grid[_LAYER_AGENTS, y, x]
                    if agent_id:
                        commited_agents.add(agent_id)

        commited_agents = set([self.agents[id_ - 1] for id_ in commited_agents])
        failed_agents = set(self.agents) - commited_agents

        for agent in failed_agents:
            assert agent.req_action == Action.FORWARD
            agent.req_action = Action.NOOP

        rewards = np.zeros(self.n_agents)

        for agent in self.agents:
            agent.prev_x, agent.prev_y = agent.x, agent.y

            if agent.req_action == Action.FORWARD:
                agent.x, agent.y = agent.req_location(self.grid_size) # TODO: move agent by method 
                # if agent.carrying_shelf:
                #     agent.carrying_shelf.x, agent.carrying_shelf.y = agent.x, agent.y
                if agent.carrying_package():
                    agent.move_container() # This method could be inside agent's 'move' method 
            elif agent.req_action in [Action.LEFT, Action.RIGHT]:
                agent.dir = agent.req_direction()
            # elif agent.req_action == Action.TOGGLE_LOAD and not agent.carrying_shelf:
            #     shelf_id = self.grid[_LAYER_SHELFS, agent.y, agent.x]
            #     if shelf_id:
            #         agent.carrying_shelf = self.shelfs[shelf_id - 1]
            elif (
                agent.req_action == Action.TOGGLE_LOAD and
                agent.container.count_packages() < self.package_carrying_capacity_per_agent and
                self._is_start_pose_with_package(agent.x, agent.y)
                ):
                # implement loading
                # if agent is in start position with requested package, load package
                loaded_package = self._load_package_from_stat_pose(agent.x, agent.y)
                agent.container.load(loaded_package)
                if self.reward_type == RewardType.TWO_STAGE:
                    rewards[agent.id -1] += 0.5
                
            # deprecate shelf returning            
            # elif agent.req_action == Action.TOGGLE_LOAD and agent.carrying_shelf:
            #     if not self._is_highway(agent.x, agent.y):
            #         agent.carrying_shelf = None
            #         if agent.has_delivered and self.reward_type == RewardType.TWO_STAGE:
            #             rewards[agent.id - 1] += 0.5
            #         agent.has_delivered = False


        self._recalc_grid()

        # shelf_delivered = False
        package_delivered = False
        # for y, x in self.goals:
        for y, x in self.requested_goals:
            # shelf_id = self.grid[_LAYER_SHELFS, x, y]
            # if not shelf_id:
            #     continue
            # shelf = self.shelfs[shelf_id - 1]

            # TODO: maybe change gridmap[_LAYER_AGENT] to incorporate multiple agent at same sell
            # NOTE: assume grid[_LAYER_AGENTS] represents one of agent id at the same cell
            # only one agent can occupy goal_pose
            agent_id =self.grid[_LAYER_AGENTS, x, y]
            if not agent_id:
                continue
            agent = self.agents[agent_id -1]


            # if shelf not in self.request_queue:
            #     continue
            # a shelf was successfully delived.
            # shelf_delivered = True
            if not agent.carrying_package():
                continue
            # a package was successfully delilved.
            package_delivered = True

            # remove from queue and replace it
            # new_request = np.random.choice(
            #     list(set(self.shelfs) - set(self.request_queue))
            # )
            # self.request_queue[self.request_queue.index(shelf)] = new_request

            # package is removed from request_queue when it was loaded from the start
            # unload package from container inside agent
            # add new package to request_queue
            new_package = self._create_new_package()
            self.request_queue.append(new_package)
            agent.container.unload()
            # delete goal and create new one
            self._replace_goal(y, x)


            # also reward the agents
            if self.reward_type == RewardType.GLOBAL:
                rewards += 1
            elif self.reward_type == RewardType.INDIVIDUAL:
                agent_id = self.grid[_LAYER_AGENTS, x, y]
                rewards[agent_id - 1] += 1
            elif self.reward_type == RewardType.TWO_STAGE:
                agent_id = self.grid[_LAYER_AGENTS, x, y]
                self.agents[agent_id - 1].has_delivered = True
                rewards[agent_id - 1] += 0.5

        # if shelf_delivered:
        if package_delivered:
            self._cur_inactive_steps = 0
        else:
            self._cur_inactive_steps += 1
        self._cur_steps += 1

        if (
            self.max_inactivity_steps
            and self._cur_inactive_steps >= self.max_inactivity_steps
        ) or (self.max_steps and self._cur_steps >= self.max_steps):
            dones = self.n_agents * [True]
        else:
            dones = self.n_agents * [False]
        
        new_obs = tuple([self._make_obs(agent) for agent in self.agents]) if not self.use_full_obs \
            else self._make_obs(self.agents[0])
        info = {}
        return new_obs, list(rewards), dones, info

    def render(self, mode="human"):
        if not self.renderer:
            from rware.rendering import Viewer

            self.renderer = Viewer(self.grid_size, self.block_size)
        return self.renderer.render(self, return_rgb_array=mode == "rgb_array")

    def close(self):
        if self.renderer:
            self.renderer.close()

    def seed(self, seed=None):
        ...
    

if __name__ == "__main__":
    from layout import layout_smallstreet, layout_2way, layout_2way_simple, layout_2way_test_giuk
    env = Warehouse(n_agents=4,
                    msg_bits=0,
                    sensor_range=3,
                    request_queue_size=5,
                    max_inactivity_steps=10000,
                    max_steps=10000,
                    reward_type=RewardType.TWO_STAGE,
                    layout=layout_2way_simple,
                    block_size="big",
                    observation_type=ObserationType.IMAGE,
                    use_full_obs=True
                    )
    env = Warehouse(n_agents=30,
                    msg_bits=0,
                    sensor_range=10,
                    request_queue_size=20,
                    max_inactivity_steps=10000,
                    max_steps=10000,
                    reward_type=RewardType.TWO_STAGE,
                    layout=layout_2way,
                    block_size="small",
                    observation_type=ObserationType.IMAGE,
                    use_full_obs=True
                    )
    import time
    from tqdm import tqdm

    # env.step(18 * [Action.LOAD] + 2 * [Action.NOOP])
    state = env.reset()

    for _ in tqdm(range(1000000)):
        # time.sleep(2)
        env.render()
        actions = env.action_space.sample()
        next_obs, reward, dones, info = env.step(actions)
