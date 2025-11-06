# Drive Constants
Drive Constants can be found in the [Constants.java](/src/main/java/us/kilroyrobotics/Constants.java) file under the DriveConstants subclass (line ~26)

Other speeds can be found in their repective sublcass (e.g. coral outtake speed in CoralMechanismConstants)

You can adjust 3/4 of the constants as needed, but do not change the slow drive speed used for slight adjustments (currently 0.5m/s).

## Original Values
- Medium - 2.0m/s
- Defense/High - 3.5m/s
- Rotation - 0.75
- Coral Outtake speed - 0.2

Rumble Notes:
-high speed was 2.0, changed to 2.5
-med speed was 1.0, changed to 1.75
-Tower.java line 346: changed readytoscore time from 0.25 sec to 2 sec
-Constants.java line 186: changed coralstationheight from 32.25 to 31.75
