# VirxERLU-RLib

+ [RLBot](http://www.rlbot.org/)
+ [VirxERLU Wiki](https://github.com/VirxEC/VirxERLU/wiki)
+ [VirxEC Discord](https://discord.gg/rutfWr4Yrw) - Ask your questions here!
+ [RLBot Wiki](https://github.com/RLBot/RLBot/wiki)
+ [RLBot Discord](https://discord.gg/rlbot)
+ [RLBot Youtube](https://www.youtube.com/channel/UCu8scB_k94Kh-iO979QTDJA)
+ [VirxEC Showcase](https://www.virxcase.dev)
+ [VirxERLU on VirxEC Showcase](https://virxerlu.virxcase.dev/)
+ [VirxEB on VirxEC Showcase](https://virxeb.virxcase.dev/)

## About

+ [Main GitHub page](https://github.com/VirxEC/VirxERLU)
+ [Cloning to another repository](https://github.com/VirxEC/VirxERLU/generate)
+ [Get the zip](https://github.com/VirxEC/VirxERLU/archive/master.zip)

VirxERLU is a series of utilities for RLBot.

VirxERLU-RLib is the high-performance Rust code with links to Python in order to provide fast and accurate numbers that you can trust.

## Features

Currently, VirxERLU-RLib has:

+ 120tps ball prediction analysis
+ Near on-point shots and distance calculations (always WIP but fairly good)
+ 120tps acceleration simulation using modified RLU speed controller
+ Pathing using Dubin's Paths that says in the field
+ Semi-variable turn radius calculations for pathing
+ 6 paths, 6 ways to stay in the field per slice
+ Shoots between two goal posts, not at a single point
+ Goal post correction
+ GameTickPacket parsing
+ Custom ball prediction struct length
+ Temporary shots (for just checking the time of a possible shot)
+ Support for any car on the field
+ Shot options
    - Search starting slice
    - Search ending slice
    - Using the car's true max speed (2300) instead of the value based off of the current boost amount
    - More coming
+ SIMD vector math

## Using the car's true max speed

This might be a little confusing, so I'm going to explain it more.

By default, every tick this library calculates the max speed that the car get get to with it's current boost amount. This has several benefits, such as being able to turn tighter on low boost and getting to locations faster. However, this is also suseptable to boost pick-ups. If you're bot picks up a small or large boost pad, the course of the bot may be entirely different after the fact.

By passing in "use_absolute_max_values" with a value of "True" this library will make sure that the path always stays the same, and it will only get faster with boost pickups. However, this may render certain shots impossible as the bot trys to say clear of walls and can't make as tight of a turn.

TL;DR enabling this makes the shots more consistant and reliable, at the cost of getting to some shots faster or even not at all (but it will know that from the start.)

POTENTIAL SOLUTION: In the max speed calculation, consider all potential boost pad pickups.
