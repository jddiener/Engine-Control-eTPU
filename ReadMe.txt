This set of engine control eTPU code is based upon the NXP AN4907 engine control library. 
Currently it provides the following above and beyond the baseline AN4907 functionality:

Enhancements:
- perform a second start angle recalculation for FUEL/SPARK that is performed closer 
  to the start than the programmed recalculation offset, thus providing more accurate 
  output timing under acceleration/deceleration conditions.
- perform a higher resolution TCR1 to TCR2 conversion when scheduling angle minus 
  time matches that generate signal edges.
- factor acceleration into the trr (tick rate register) in order to provide more 
  accurate engine position to the output timing functions under dynamic conditions.

Bug fixes:
- the FUEL and SPARK functions had startup issues that could lead to inaccurate 
  output at the start of engine sync.
- the FUEL injection applied time was not calculated correctly if the stop angle was hit.
- the trr (tick rate register) value was not being calculated correctly when TCR1 was 
  configured to use the system clock as its source, leading to some inaccuracy in engine 
  position and thus also fuel/spark/injection timing. 
- the SPARK max dwell capability did not work, nor did the min dwell fault detection work 
  in all cases.
- fix a stall issue wherein there was a small window of TCR2 values which would result in 
  serious FUEL/SPARK/INJ/KNOCK misbehavior (output signals could fire while 
  re-synchronization still in progress).
- fix an issue where after synchronization there can be a small phase shift between actual 
  engine angle and reported angle, which remains constant until a re-synchronization
  occurs.
