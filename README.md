# Lunar Transfer Planner

### Provides a GUI for planning a transfer to a satellite (natural or artificial) from the surface of the planet on which you are currently on.

#### Usecases include:
* Launching to the Moon from any position on Earth
* Launching to the Mun or Minmus from any position on Kerbin
* Rendezvousing with a vessel in orbit

#### Parameters that you can change:
* The expected flight time to the satellite (the time from leaving your circular parking orbit to reaching the closest approach to your target)
* The altitude of your parking orbit
* The target launch azimuth or inclination for the launch window, with a default of 90° azimuth (directly due east)
* The latitude and longitude of your position
    * This can be given automatically by the position of your launch site or vessel, or by manually typing in a latitude and longitude

#### Info given back to help you plan:
* The time of the next launch window to your parking orbit at the target azimuth/inclination
    * Also shows the nth launch window from the current time, where n is user-defined (max of 100, but this can be changed in settings)
* The delta-V of the maneuver from your parking orbit to an intersection with the target
* The time spent in your parking orbit while waiting for the maneuver
    * Can also show the phasing angle from the current position

Keep in mind that this mod assumes you can instantly get into the target orbit from your current launch position. If you want more accurate results (especially for launching into a vessel's orbit), launch into the next window's inclination a few minutes before the window itself, with the time depending on how long it takes you to reach orbit.

Forum Thread: N/A

Source Code: https://github.com/KSP-RO/LunarTransferPlanner

## Mod Relationships

#### Required:
* ClickThroughBlocker
* ToolbarController

#### Recommended:
* Kerbal Alarm Clock (required for "Add Alarm" button)

#### Compatible with:
* Principia (specifically, the changing inclination of the target's orbit due to Principia)
* Any celestial body from any solar system (including satellites of moons, and technically even from the Surface of the Sun/Kerbol?)

## Authors
RCrockford

## License
The code is subject to the MIT license (see below). 

---

Copyright (c) 2022 RCrockford

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

---
