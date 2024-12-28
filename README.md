# Engine-Control-eTPU
This project is a set of eTPU engine control software based upon the NXP AN4907 engine control library.  It contains several enhancements and fixes several minor issues - see the ReadMe.txt file for more details.  This is a work in progress and it is expected we at ASH WARE or others will add to the project as time goes on.

This software is built and simulated/tested by the following tools:
- ETEC C Compiler for eTPU/eTPU2/eTPU2+, version 3.00A, ASH WARE Inc.
- eTPU2+ Development Tool, version 2.74A, ASH WARE Inc.
- System Development Tool, version 2.74A, ASH WARE Inc.

Use of or collaboration on this project is welcomed. For any questions please contact:

ASH WARE Inc. John Diener john.diener@ashware.com

# Release Information

## 1.1.0

- fix an issue where an initial FUEL pulse after synchonization could have incorrect timing and be much too long
- fix an issue where noise on the crank signal could result in a phase shift of the engine position which would only correct on re-synchronization

## 1.0.1

- fix an issue where after synchronization there can be a small phase shift between actual 
  engine angle and reported angle, which remains constant until a re-synchronization
  occurs.

## 1.0.0

- initial release with several bug fixes and enhancements over the standard AN4907 code