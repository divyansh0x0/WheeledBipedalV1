import pygame

from linkage import World, Link

world = World.World()
world.add(Link.Link(150))
world.add(Link.Link(150))
world.add(Link.Link(80))
world.add(Link.Link(113.3))
world.add(Link.Link(60))
world.start_loop()

