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
+ Support for any car on the field
+ Shot options
    - Search starting slice
    - Search ending slice
    - More coming
+ SIMD vector math
