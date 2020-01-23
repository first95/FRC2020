# Software full acceptance test

This test should be fully run, and should fully pass, prior to merging into master.

It's very important to follow the instructions quite literally and exactly.  If there's a mistake in the test, we should fix the test.  This is to make sure we're running the same test every time.

Write a check mark or "P" for every test that passes.  Write an "F" for any test that fails, and write a note describing the failure.

## Teleoperated feature check.

Here we exercise every feature of the robot that can be used during teleoperated mode.  If any of these fail, go back and fix the software, then restart the test from the start.

### Drivebase 

- [ ] Drive forward and backward
- [ ] Manually shift gears and confirm you can hear shifting. (A loud click)
- [ ] Drive fast and confirm the robot upshifts automatically. (You should hear a loud click)
- [ ] Let the robot slow down and confirm the robot downshifts automatically. (You should hear a loud click)

## Startup test.

- [ ] Set up a "practice" in the drivers' station.  This will run an auto move, then transition into teleoperated.  Confirm that the auto move runs, and that no exceptions are thrown on the console when the robot enters teleoperated mode.

## Autonomous mode checks

Here we test our autonomous moves.

## Regression tests

Here we run a test for each major bug we've found.  This is to make sure the bug has remained fixed.

