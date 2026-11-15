# Questionnaire: what the twin is still guessing

Answer what you can, by number. Each answer replaces a guess in the Calibration panel and
that value stops being varied in trials. `*` marks the answers that change the twin's
predictions most. "Now" is the value the twin currently uses.

## A. Course

1. Is the corridor 30 cm wide in every leg, measured wall face to wall face? (Now: 30 everywhere, as you said.)
2. `*` With a constant width, the brief's outer and inner wall lengths cannot both be right. Measure the inner wall of the second leg (the one after the first bend). Brief: 51 cm. Twin, built from the outer wall: 45.7 cm. Which is it nearer?
3. What are the walls made of, and are they matte or glossy? (Now: matte board, from one photo.)
4. Wall thickness? (Now: 0.6 cm.)
5. Wall height, and height of the ultrasonic sensors above the floor? (Not modelled; matters only if the sensors look over the walls.)
6. Where exactly do you place the robot at the start: wheels on the line, tail on the line, or nose on the line? (Now: tail on the line, axle 12 cm past it.)
7. Is the start open behind the robot, or is there a back wall?

## B. Task Decider

8. `*` Distance from the end of the outer (left) wall to the Decider's face. (Now: 13 cm, from the brief. At 9 cm the twin's v21 fails to stop correctly.)
9. `*` Width and depth of the Decider as the robot sees it. (Now: 24 x 14 cm, scaled from the brief's figure.)
10. `*` Is the Decider centred across the corridor, or against one wall? (Now: centred.)
11. Does the inner (right) wall run further than the outer wall at the end, and by how much? (Now: 5 cm further.)
12. How far in front of the Decider does the flag come, and from how far away does your IR sensor detect it? (Now: detected within 20 cm.)
13. How long after the robot stops does the flag appear, and is that triggered by a person? (Now: 1.5 s.)
14. Are the flag times exactly 3, 5 and 10 s? (Now: yes, +/-0.2 s.)
15. Does the IR sensor ever trigger on the Decider's body with no flag shown?

## C. Robot, with a ruler

16. `*` Overall width at the widest point, across the two angled sensors. (Now: 19.0 cm.)
17. Width over the outside of the tyres. (Now: 15.2 cm.)
18. `*` Wheel spacing, centre of one tyre to centre of the other. (Now: 12.6 cm. Sets the spin angle.)
19. Axle line to the face of the front sensor. (Now: 8.5 cm.)
20. Axle line to the rearmost point of the robot. (Now: 12.0 cm.)
21. Side sensors: how far ahead of the axle line is the centre of each sensor's face, and how far from the robot's centreline? (Now: 3.3 cm ahead, 7.6 cm out.)
22. Are the side sensors at 45 degrees to straight ahead? (Now: 45.)
23. IR sensor: how far ahead of the axle line? (Now: 6.0 cm.)

## D. Drive (use trim_test_v3.py for 24 to 26)

24. `*` Distance covered in 3.0 s at 40 % power on the course floor. (Twin: about 57 cm.)
25. Same at 55 %. (Twin: about 96 cm.)
26. Sideways drift over that 3 s run with the current trims, and to which side. (Now: under 2 cm either way.)
27. `*` Angle turned by the spin task exactly as coded in v21 (1.5 s at 55 %). (Twin: about 373 degrees.)
28. Lowest power at which a wheel starts moving from rest with the robot on the floor. (Now: 23 %.)
29. Lowest power at which a wheel that is already turning keeps turning. (Now: 18 %.)
30. How far does the robot roll on after power is cut at 40 %? (Twin: about 1.3 cm.)
31. Does a fresh set of batteries make it noticeably faster than a used set? (Now: +/-4 %.)

## E. Sensors (use ultrasonic_test_v4.py, robot standing in the corridor)

32. `*` Robot centred and parallel in a 30 cm leg: what do the left and right sensors read? (Twin: about 8.8 cm each. Nearer 10.5 cm would mean the sensors measure straight along their axis and the twin's beam is still too wide.)
33. Same, with the robot moved 3 cm towards the right wall.
34. `*` Robot standing just before a bend, where a side sensor looks down the next leg: does that sensor read a distance or nothing? (The log says "stopped at every corner where the track widens" for v11a, which suggests nothing.)
35. Front sensor square-on to a wall at 10, 20 and 40 cm: readings?
36. Front sensor facing a wall turned 30 degrees, then 45 degrees, from 15 cm: does it read, and what?
37. Closest distance at which the front sensor still gives a sensible reading. (Now: 2 cm.)
38. With a wall plainly in view, how many pings in 100 come back with no echo? (Now: 1.)
39. Any false short readings from the front sensor on a clear straight? (Now: about one per run with the 5 ms gap.)

## F. v21 on the real course

40. `*` Drive time from first movement to the stop at the Decider. Was your "about 20 s" timed from switch-on or from first movement? (Twin: 17.4 s driving, 20.4 s from switch-on.)
41. `*` Does it touch a wall anywhere? Where?
42. How far from the Decider does it stop, and is it square to it? (Twin: 9 cm, square.)
43. Out of how many runs has it completed? Any failures, and what happened?
44. Does it ever pivot on one wheel at a bend, or always steer smoothly? (Twin: smooth.)
45. Which of the three tasks have been tested on the real Decider, and with what result?

## G. Earlier versions

46. `*` By "run 10" and "run 13", do you mean trials 10 and 13 in the testing log, or code versions v10 and v13? The log has trial 10 (code v9) as rejected and trial 13 (code v8a) as completing.
47. Which versions do you remember completing the whole course? The twin's Log check uses: v8, v8a, v11, v12 and v21 complete; v9 and v11h do not; v11a, v11e and v11g stop early; v12 does not stop at the Decider. Correct anything wrong.
48. When v12 "did not stop at the Decider", what did it do there: push against it, turn away, or keep pivoting?
49. Was the left sensor really returning nothing for every run before v13b, or only in the logged run?
50. On what date was the right motor turned round? (Now assumed 11 Nov, between v13b and v13c.)
