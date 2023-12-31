"""
2D rendering of the Robotic's Warehouse
environment using pyglet
"""

import math
import os
import sys

import numpy as np
import math
import six
from gym import error
from rdlvy.robot_delivery import Direction

if "Apple" in sys.version:
    if "DYLD_FALLBACK_LIBRARY_PATH" in os.environ:
        os.environ["DYLD_FALLBACK_LIBRARY_PATH"] += ":/usr/lib"
        # (JDS 2016/04/15): avoid bug on Anaconda 2.3.0 / Yosemite


try:
    import pyglet
except ImportError as e:
    raise ImportError(
        """
    Cannot import pyglet.
    HINT: you can install pyglet directly via 'pip install pyglet'.
    But if you really just want to install all Gym dependencies and not have to think about it,
    'pip install -e .[all]' or 'pip install gym[all]' will do it.
    """
    )

try:
    from pyglet.gl import *
except ImportError as e:
    raise ImportError(
        """
    Error occured while running `from pyglet.gl import *`
    HINT: make sure you have OpenGL install. On Ubuntu, you can run 'apt-get install python-opengl'.
    If you're running on a server, you may need a virtual frame buffer; something like this should work:
    'xvfb-run -s \"-screen 0 1400x900x24\" python <your_script.py>'
    """
    )


RAD2DEG = 57.29577951308232
# # Define some colors
_BLACK = (0, 0, 0)
_WHITE = (255, 255, 255)
_DARKORANGE = (255, 178, 102)
_GREY = (160, 160, 160)
_LIGHTGERY = (192, 192, 192)

_BACKGROUND_COLOR = _WHITE
_GRID_COLOR = _LIGHTGERY 
_SHELF_COLOR = _GREY
_SHELF_REQ_COLOR =  (154, 0, 76) 
_AGENT_COLOR = _DARKORANGE
_AGENT_WITH_PACKAGE_3= (153, 0, 76) 
_AGENT_WITH_PACKAGE_2= (255, 0, 127) 
_AGENT_WITH_PACKAGE_1= (255, 153, 204)
_AGENT_DIR_COLOR = _BLACK
_GOAL_COLOR = (102, 102, 255)

_START_CANDIDATE_COLOR = (255, 204, 204)
_REQUESTED_PACKAGE_COLOR =  (255, 0, 127)
_GOAL_CANDIDATE_COLOR = _GREY 

_START_CAND_PADDING = 0
_GOAL_CAND_PADDING = 0.8
_SHELF_PADDING = 0.8 
_PACKAGE_PADDING = 2



def get_display(spec):
    """Convert a display specification (such as :0) into an actual Display
    object.
    Pyglet only supports multiple Displays on Linux.
    """
    if spec is None:
        return None
    elif isinstance(spec, six.string_types):
        return pyglet.canvas.Display(spec)
    else:
        raise error.Error(
            "Invalid display specification: {}. (Must be a string like :0 or None.)".format(
                spec
            )
        )


class Viewer(object):
    def __init__(self, world_size, block_size="small"):
        display = get_display(None)
        self.rows, self.cols = world_size

        self.grid_size = 16 if block_size=="small" else 30
        self.icon_size = 15 if block_size=="small" else 20

        self.width = 1 + self.cols * (self.grid_size + 1)
        self.height = 1 + self.rows * (self.grid_size + 1)
        self.window = pyglet.window.Window(
            width=self.width, height=self.height, display=display
        )
        self.window.on_close = self.window_closed_by_user
        self.isopen = True

        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

    def close(self):
        self.window.close()

    def window_closed_by_user(self):
        self.isopen = False
        exit()

    def set_bounds(self, left, right, bottom, top):
        assert right > left and top > bottom
        scalex = self.width / (right - left)
        scaley = self.height / (top - bottom)
        self.transform = Transform(
            translation=(-left * scalex, -bottom * scaley), scale=(scalex, scaley)
        )

    def render(self, env, return_rgb_array=False):
        glClearColor(*_BACKGROUND_COLOR, 0)
        self.window.clear()
        self.window.switch_to()
        self.window.dispatch_events()

        self._draw_grid()
        self._draw_shelfs(env)
        self._draw_start_candidates(env)
        self._draw_requested_packages(env)
        self._draw_goal_candidates(env)
        self._draw_goals(env)
        self._draw_agents(env)

        if return_rgb_array:
            buffer = pyglet.image.get_buffer_manager().get_color_buffer()
            image_data = buffer.get_image_data()
            arr = np.frombuffer(image_data.get_data(), dtype=np.uint8)
            arr = arr.reshape(buffer.height, buffer.width, 4)
            arr = arr[::-1, :, 0:3]
        self.window.flip()
        return arr if return_rgb_array else self.isopen

    def _draw_grid(self):
        batch = pyglet.graphics.Batch()
        # VERTICAL LINES
        for r in range(self.rows + 1):
            batch.add(
                2,
                gl.GL_LINES,
                None,
                (
                    "v2f",
                    (
                        0,  # LEFT X
                        (self.grid_size + 1) * r + 1,  # Y
                        (self.grid_size + 1) * self.cols,  # RIGHT X
                        (self.grid_size + 1) * r + 1,  # Y
                    ),
                ),
                ("c3B", (*_GRID_COLOR, *_GRID_COLOR)),
            )

        # HORIZONTAL LINES
        for c in range(self.cols + 1):
            batch.add(
                2,
                gl.GL_LINES,
                None,
                (
                    "v2f",
                    (
                        (self.grid_size + 1) * c + 1,  # X
                        0,  # BOTTOM Y
                        (self.grid_size + 1) * c + 1,  # X
                        (self.grid_size + 1) * self.rows,  # TOP Y
                    ),
                ),
                ("c3B", (*_GRID_COLOR, *_GRID_COLOR)),
            )
        batch.draw()

    def _draw_shelfs(self, env):
        batch = pyglet.graphics.Batch()

        for shelf in env.shelfs:
            x, y = shelf.x, shelf.y
            y = self.rows - y - 1  # pyglet rendering is reversed
            shelf_color = (
                _SHELF_REQ_COLOR if shelf in env.request_queue else _SHELF_COLOR
            )

            batch.add(
                4,
                gl.GL_QUADS,
                None,
                (
                    "v2f",
                    (
                        (self.grid_size + 1) * x + _SHELF_PADDING + 1,  # TL - X
                        (self.grid_size + 1) * y + _SHELF_PADDING + 1,  # TL - Y
                        (self.grid_size + 1) * (x + 1) - _SHELF_PADDING,  # TR - X
                        (self.grid_size + 1) * y + _SHELF_PADDING + 1,  # TR - Y
                        (self.grid_size + 1) * (x + 1) - _SHELF_PADDING,  # BR - X
                        (self.grid_size + 1) * (y + 1) - _SHELF_PADDING,  # BR - Y
                        (self.grid_size + 1) * x + _SHELF_PADDING + 1,  # BL - X
                        (self.grid_size + 1) * (y + 1) - _SHELF_PADDING,  # BL - Y
                    ),
                ),
                ("c3B", 4 * shelf_color),
            )
        batch.draw()

    def _draw_goals(self, env):
        batch = pyglet.graphics.Batch()

        for goal in env.requested_goals:
            x, y = goal
            y = self.rows - y - 1  # pyglet rendering is reversed
            batch.add(
                4,
                gl.GL_QUADS,
                None,
                (
                    "v2f",
                    (
                        (self.grid_size + 1) * x + 1,  # TL - X
                        (self.grid_size + 1) * y + 1,  # TL - Y
                        (self.grid_size + 1) * (x + 1),  # TR - X
                        (self.grid_size + 1) * y + 1,  # TR - Y
                        (self.grid_size + 1) * (x + 1),  # BR - X
                        (self.grid_size + 1) * (y + 1),  # BR - Y
                        (self.grid_size + 1) * x + 1,  # BL - X
                        (self.grid_size + 1) * (y + 1),  # BL - Y
                    ),
                ),
                ("c3B", 4 * _GOAL_COLOR),
            )
        batch.draw()
    
    def _draw_goal_candidates(self, env):
        batch = pyglet.graphics.Batch()

        for goal in env.goal_candidates:
            x, y = goal
            y = self.rows - y - 1  # pyglet rendering is reversed
            batch.add(
                4,
                gl.GL_QUADS,
                None,
                (
                    "v2f",
                    (
                        (self.grid_size + 1) * x + _GOAL_CAND_PADDING + 1,  # TL - X
                        (self.grid_size + 1) * y + _GOAL_CAND_PADDING + 1,  # TL - Y
                        (self.grid_size + 1) * (x + 1) - _GOAL_CAND_PADDING,  # TR - X
                        (self.grid_size + 1) * y + _GOAL_CAND_PADDING + 1,  # TR - Y
                        (self.grid_size + 1) * (x + 1) - _GOAL_CAND_PADDING,  # BR - X
                        (self.grid_size + 1) * (y + 1) - _GOAL_CAND_PADDING,  # BR - Y
                        (self.grid_size + 1) * x + _GOAL_CAND_PADDING + 1,  # BL - X
                        (self.grid_size + 1) * (y + 1) - _GOAL_CAND_PADDING,  # BL - Y
                    ),
                ),
                ("c3B", 4 * _GOAL_CANDIDATE_COLOR),
            )
        batch.draw()

    def _draw_requested_packages(self, env):
        batch = pyglet.graphics.Batch()

        for package in env.request_queue:
            x, y = package.x, package.y
            y = self.rows - y - 1  # pyglet rendering is reversed
            batch.add(
                4,
                gl.GL_QUADS,
                None,
                (
                    "v2f",
                    (
                        (self.grid_size + 1) * x + 1,  # TL - X
                        (self.grid_size + 1) * y + 1,  # TL - Y
                        (self.grid_size + 1) * (x + 1),  # TR - X
                        (self.grid_size + 1) * y + 1,  # TR - Y
                        (self.grid_size + 1) * (x + 1),  # BR - X
                        (self.grid_size + 1) * (y + 1),  # BR - Y
                        (self.grid_size + 1) * x + 1,  # BL - X
                        (self.grid_size + 1) * (y + 1),  # BL - Y
                    ),
                ),
                ("c3B", 4 * _REQUESTED_PACKAGE_COLOR),
            )
        batch.draw()

    def _draw_start_candidates(self, env):
        batch = pyglet.graphics.Batch()

        for start_candidate in env.start_candidates:
            x, y = start_candidate
            y = self.rows - y - 1  # pyglet rendering is reversed
            batch.add(
                4,
                gl.GL_QUADS,
                None,
                (
                    "v2f",
                    (
                        (self.grid_size + 1) * x + _START_CAND_PADDING + 1,  # TL - X
                        (self.grid_size + 1) * y + _START_CAND_PADDING + 1,  # TL - Y
                        (self.grid_size + 1) * (x + 1) - _START_CAND_PADDING,  # TR - X
                        (self.grid_size + 1) * y + _START_CAND_PADDING + 1,  # TR - Y
                        (self.grid_size + 1) * (x + 1) - _START_CAND_PADDING,  # BR - X
                        (self.grid_size + 1) * (y + 1) - _START_CAND_PADDING,  # BR - Y
                        (self.grid_size + 1) * x + _START_CAND_PADDING + 1,  # BL - X
                        (self.grid_size + 1) * (y + 1) - _START_CAND_PADDING,  # BL - Y
                    ),
                ),
                ("c3B", 4 * _START_CANDIDATE_COLOR),
            )
        batch.draw()

    def _draw_agents(self, env):
        agents = []
        batch = pyglet.graphics.Batch()

        radius = self.grid_size / 2.5#3

        resolution = 6

        for agent in env.agents:

            col, row = agent.x, agent.y
            row = self.rows - row - 1  # pyglet rendering is reversed

            # make a circle
            verts = []
            for i in range(resolution):
                angle = 2 * math.pi * i / resolution
                x = (
                    radius * math.cos(angle)
                    + (self.grid_size + 1) * col
                    + self.grid_size // 2
                    + 1
                )
                y = (
                    radius * math.sin(angle)
                    + (self.grid_size + 1) * row
                    + self.grid_size // 2
                    + 1
                )
                verts += [x, y]
            circle = pyglet.graphics.vertex_list(resolution, ("v2f", verts))

            if not agent.carrying_package():
                draw_color = _AGENT_COLOR
            elif agent.container.count_packages() == 3:
                draw_color = _AGENT_WITH_PACKAGE_3
            elif agent.container.count_packages() == 2:
                draw_color = _AGENT_WITH_PACKAGE_2
            elif agent.container.count_packages() == 1:
                draw_color = _AGENT_WITH_PACKAGE_1
            else:
                draw_color = _BLACK

            glColor3ub(*draw_color)
            circle.draw(GL_POLYGON)

        for agent in env.agents:

            col, row = agent.x, agent.y
            row = self.rows - row - 1  # pyglet rendering is reversed

            batch.add(
                2,
                gl.GL_LINES,
                None,
                (
                    "v2f",
                    (
                        (self.grid_size + 1) * col
                        + self.grid_size // 2
                        + 1,  # CENTER X
                        (self.grid_size + 1) * row
                        + self.grid_size // 2
                        + 1,  # CENTER Y
                        (self.grid_size + 1) * col
                        + self.grid_size // 2
                        + 1
                        + (
                            radius if agent.dir.value == Direction.RIGHT.value else 0
                        )  # DIR X
                        + (
                            -radius if agent.dir.value == Direction.LEFT.value else 0
                        ),  # DIR X
                        (self.grid_size + 1) * row
                        + self.grid_size // 2
                        + 1
                        + (
                            radius if agent.dir.value == Direction.UP.value else 0
                        )  # DIR Y
                        + (
                            -radius if agent.dir.value == Direction.DOWN.value else 0
                        ),  # DIR Y
                    ),
                ),
                ("c3B", (*_AGENT_DIR_COLOR, *_AGENT_DIR_COLOR)),
            )
        batch.draw()

    def _draw_badge(self, row, col, level):
        resolution = 6
        radius = self.grid_size / 5

        badge_x = col * self.grid_size + (3 / 4) * self.grid_size
        badge_y = self.height - self.grid_size * (row + 1) + (1 / 4) * self.grid_size

        # make a circle
        verts = []
        for i in range(resolution):
            angle = 2 * math.pi * i / resolution
            x = radius * math.cos(angle) + badge_x
            y = radius * math.sin(angle) + badge_y
            verts += [x, y]
        circle = pyglet.graphics.vertex_list(resolution, ("v2f", verts))
        glColor3ub(*_BLACK)
        circle.draw(GL_POLYGON)
        glColor3ub(*_WHITE)
        circle.draw(GL_LINE_LOOP)
        label = pyglet.text.Label(
            str(level),
            font_name="Times New Roman",
            font_size=12,
            x=badge_x,
            y=badge_y + 2,
            anchor_x="center",
            anchor_y="center",
        )
        label.draw()
