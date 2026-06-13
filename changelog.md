- Allow for force-loading sub-levels (`/sable forceload <add|remove>`)
- Catch Sable Rapier panics nicely and crash the game normally (#477)
- Update Sodium compatibility to 0.8.13 alpha
- Update NeoForge version

- Add a config for sub-level saving messages
- Add a client config for attempt_udp_networking
- Add sub_levels_with_players_cannot_unload config
- Redo sub-level explosion handling

- Separate sable_rapier into its own module
- Update the physics constraint API
- Mark some internal Sable classes as internal

Fixes
- Fix Create rope pulley rope blocks not being weightless
- Fix some visual issues with Sodium on AMD machines
- Fix spectators weirdly inheriting sub-level velocity
- Fix nether portal linking issues 
- Fix issues with slab and ladder masses
- Fix issues with loyalty tridents and sub-levels
- Fix sub-level assembly not working in short dimensions
- Fix mechanical arms thinking they aren't loaded on sublevels
- Fix track graph visualizer crash
- Fix Breeze charges not affecting contraptions (#901)
- Fix POIs not getting updated with assembly
- Fix physics property crash with some modded blocks (#685)
- Fix party parrot on sublevels
- Fix wind charge effect not considered wind charged
- Fix small issues with particles and sub-levels
- Fix standing up from crawling (#929)
- Fix item duplication with some block entities on assembly (#981)