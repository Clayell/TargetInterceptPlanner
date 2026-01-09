[![GitHub Downloads (all assets, all releases)](https://img.shields.io/github/downloads/Clayell/TargetInterceptPlanner/total)](https://github.com/Clayell/TargetInterceptPlanner/releases/latest)

# Target Intercept Planner (TIP)

### Provides a GUI for planning a transfer to a satellite (natural, artificial, or a specified orbit) while on the surface of a celestial body. (planet, moon, gas giant, star, black hole, etc.)

Launch into the parking orbit it gives you at the time it gives you, wait the time it gives you for the prograde-only maneuver to arrive, and then execute the maneuver with the delta-V it gives you. If all goes well, you should be headed directly on a collision course to your target.

Why use this over simply launching into the same LAN and Inclination as your target? This mod allows you to launch from any position on the planet, instead of being limited to a certain area on the planet. This also means that the trajectories this mod gives may not be in the same inclination of your target (because this might not be possible to achieve from your launch location), so a plane change maneuver may be needed at the intersection point when rendezvousing with a vessel.

#### Usecases include:
* Launching to the Moon from any position on Earth
* Launching to the Mun or Minmus from any position on Kerbin
* Rendezvousing with a vessel in any orbit (a plane change may be needed at the intersection point)
* Launching into an orbit given by a contract, even if no vessel is present there

#### Parameters that you can change:
* The expected flight time to the satellite (the time from leaving your circular parking orbit to reaching the closest approach to your target)
* The altitude of your parking orbit
* The target launch azimuth or inclination for the launch window, with a default of 90° azimuth (directly due east)
* The latitude and longitude of your position
    * This can be given automatically by the position of your launch site or vessel, or by manually typing in a latitude and longitude
* The exact parameters of the target orbit (in manual target orbit mode)

#### Info given back to help you plan:
* The time of the next launch window to your parking orbit at the target azimuth/inclination
    * Also shows the nth launch window from the current time, where n is user-defined (max of 100, but this can be changed in settings)
* A maneuver node from your parking orbit to an intersection with the target
* A maneuver node from your intersect trajectory to rendezvous with the target
* The time spent in your parking orbit while waiting for the maneuver
    * Can also show the phasing angle from the current position
* Map view visualizations of the parking orbit, transfer orbit, and phasing angle (and target orbit if using a manual target)

Keep in mind that this mod currently assumes you can instantly get into the parking orbit from your current launch position. If you want more accurate results (especially for launching into a vessel's orbit), launch into the next window's inclination a few minutes before the window itself, with the time depending on how long it takes you to reach orbit. You may want to use mechjeb to launch into the requested inclination, to account for the initial velocity caused by the rotation of the planet.

Note: As the orbital period of the target approaches the sidereal day length of the planet, the time to the next window approaches infinity. It is impossible to find a launch window for a target in a synchronous orbit using this tool. Sorry!

Forum Thread: N/A

Source Code: https://github.com/Clayell/TargetInterceptPlanner

Wiki: https://github.com/Clayell/TargetInterceptPlanner/wiki

## Mod Relationships (using [CKAN](https://github.com/KSP-CKAN/CKAN) is highly recommended)

#### Required:
* ClickThroughBlocker
* Harmony2
* ToolbarController

#### Recommended:
* Kerbal Alarm Clock (the "Add Alarm" button uses KAC by default if installed)

#### Compatible with:
* Principia (although not perfectly accurate due to n-body stuff)
* Any celestial body from any solar system (including satellites of moons, and technically even from the surface of stars)

## Authors
* [RCrockford](https://github.com/RCrockford) (author of original Lunar Transfer Planner)
* [Clayel](https://github.com/Clayell) (current author of Target Intercept Planner)

## Special Thanks To
* [Nazfib](https://github.com/Nazfib) ([Southern Latitude fix](https://github.com/KSP-RO/LunarTransferPlanner/pull/8) and Orbit/Angle Renderer from [TWP2](https://github.com/Nazfib/TransferWindowPlanner2))
* test_account ([Delta-V and phasing time calculations](https://github.com/KSP-RO/LunarTransferPlanner/pull/3))
* [siimav](https://github.com/siimav) ([Tooltips from RP-1](https://github.com/KSP-RO/RP-1/blob/master/Source/RP0/UI/Tooltip.cs))
* [TriggerAu](https://github.com/TriggerAu) ([KAC Wrapper](https://github.com/TriggerAu/KerbalAlarmClock/blob/master/KerbalAlarmClock/API/KACWrapper.cs))

## License
The code is subject to the MIT license. (see [LICENSE.md](https://github.com/Clayell/TargetInterceptPlanner/blob/master/LICENSE.md))

![Intercepting with the Moon](https://i.imgur.com/VRTycpy.png)
![Intercepting with a vessel](https://i.imgur.com/KgqsGP2.png)
![Intercepting with a manually set orbit](https://i.imgur.com/utW7UJI.png)
![Intercepting with the Mun with a hyperbolic transfer](https://i.imgur.com/a8JggTj.png)