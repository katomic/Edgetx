# OTXSim 
This lua script was tested on the Companion application and a Radiomaster TX15 ONLY! 

Add this in its own folder under the WIDGETS Folder on the SD or card or the SD folder for the Companion (I used 2.12.5 to test this)

In EdgeTx, select your model, and go to the Edit UI screen.
Add a full screen page, then add this widget onto that. 

Settings:
Select initial view ( 1: 3D, 2: Rear, 3: Side, 4: Top)
Show control surface labels
Select THROTTLE channel
Select color for wings, fuselage, tail and control surfaces

Exit, and press PageUp or PageDn to view the Widget. 
Tap and hold , select Full Screen.

There are 4 views to choose from: 3D, Rear, Side and Top.
Wiggle your controls to see the animation on-screen. 

In Full screen view, you can tap Assign to change the visual assignments. 
In my FliteTest SimpleScout, the two Aileron channels are initial rendered to be moving both up or down instead of opposite. Reverse one here to fix such an issue.

**Important** Changes in this Assignment screen does NOT affect your mixes, only the visuals in to screen. 

Each model's assignments are saved in the OTXSim folder, so it it will persist between sessions.



2026-11-19:
New: 
1. Propeller now shown as stopped when throttle is less than -99%.
2. Color picker for parts added.
3. Control surfaces rendered larger, landing gear removed.
4. Fuseleage shape simplified.