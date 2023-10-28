.. _userModulesTaskRunnerManager:

Task Runner manager module
===================================

The :ref: `Task Runner Manager <userModulesTaskRunnerManager>` module allows the sequencing of multiple protocols within the task runner. The 
Manager understands a subset of commands that permits automation of some data acquistion tasks.

The available tasks to be sequenced are specified in the configuration file as follows::

    TaskRunnerManager:
        SequenceGroup1:
            @ CCIV_long_1nA_max
            wait 10s
            @ Led_flasher_VC_10Hz
            wait 10s
            ZeroTime
            Stim0 = 25 uA
            @ baselineMonitor

        SequenceGroup2:
            repeat N
            interval = 300s
            @ CCIV_long_1nA_max

        SequenceGroup3:
            run SequenceGroup1
            wait 5s
            run SequenceGroup2


When started, the module brings up a separate window with two drop down lists, and checks to see that the taskmanager window
is loaded. The module also then confirms that the syntax of the sequencegroup is acceptable, the entered values parse correctly, 
and that the protocols exist. The sequence is "silently" executed (protocols are loaded but not run), with all wait times set to 1ms

* The first list includes all of the SequenceGroups that have been specified. The elements of this 
list can be selected one at a time.
* The second list will be updated to show the tasks that will be run when the sequence group is started. This list is
not editable, but is what was specified in the original configuration file. Protocols are prefixed with an @ sign. they
are executed in the order specified. Between protocols, you may specify the interval to the next protocol

There are some special commands that can be inserted into the sequencer:
    * @ protocolname : Loads and Runs the specified protocol immediately. Protocols in subdirectories may be specified
    with a path-like syntax (e.g., "myMappingExperiments/CCIV_001")
    * repeat N : optional. This command will cause the remaining tasks in the SequenceGroup to be run N times. Omission of this 
    command is the same as "repeat 1"
    * wait: float with units, optional. This specifies the wait time before executing the next protocol. This time is in addition to any 
    intervals that may already be in the protocols. You should check to be sure that this time is appropriate. If no wait is
    specified, a value of 1 second is interspersed between successive protocols.
    * run sequencegroup : This causes other sequencegroups to be run. You should be careful with this command to avoid
    infinite loops or excessively long data collections. 
    * Zerotime  - sets the "zero time" for subsequent starting intervals in conjunctions with "onInterval"
    * onInterval  This specifies that after a protocol ends, the next one will start at a particular time interval relative
    to the time noted when "zerotime" is specified. This may be useful for running multiple protocols while keeping the timing of measurements
    close to constant (e.g., 1 minute intervals when doing a pharamcology manipulation). This overrides any "wait" command.



Below the list are several buttons:
    * Start : starts the sequence run. The run will go to completion.
    * Stop : stops the sequence run immediately. The sequence cannot be continued, but only restarted. If the run is in "Pause" mode
    then, the sequence is terminated.
    * Pause: Pauses the sequence run. This should be rarely used as it may affect analysis. 
    * Continue: Only valid with the sequence has been paused, otherwise it is ignored.
    * Quit: Stop any ongoing sequence run and close the TaskRunnerManager window.


Internals:
    The sequencer is processed by an internal finite state machine. 

Notes:
    Some protocols, such as those that perform laser scanning maps, require user intervention to set up the maps
    prior to execution. These protocols are flagged as being unusable. 
    Future idea: Allow the user to set up the map in advance, and "save" the positions to a file. The
    "@ protocol" could then have a position file argument, perhaps as an editable field in the TaskRunnerManager.
    The position file would load the mapping information.  I am not sure however that the use case for this is
    very broad. Another would be to force the protocol to pause while the user builds the map, then let them
    continue. Another would be to have "standard" maps (of a specified size) that can be loaded, and then changed
    by only position and rotation.


Controls:

* **Enable** Determines whether the analysis is enabled. If checked, then every task that is executed will be recorded (in memory) and analyzed for evoked events. This analysis is represented as a spot displayed in a :ref:`camera module <userModulesCamera>`, and as a new entry in the list below.
* **Camera Module** Selects the camera module in which to display analysis results.
* **Scanner Device** Selects the scanner device that determines the location of photostimulation flashes during the task.
* **Clamp Device** Selects the electrode amplifier that will be used to measure evoked events.
* **Delete** Causes the currently selected analysis result to be deleted from memory and removed from the camera module.
* **Clamp Baseline** sets the beginning and end times of the baseline period to measure from the clamp trace. Typically this should be defined from t=0 until immediately before the photostimulation.
* **Clamp Test** sets the begining and end times of the test period to measure from the clamp trace. Typically this is defined from a few ms following the photostimulation until 10-50 ms later. 
* **Spike Threshold** The recording amplitude required to be considered a "spike". 
* **Abs / Rel** Determines whether the spike threshold is *absolute* (in volts relative to ground), or *relative* to the median value of the baseline period.
* **Color Mapper**
* **Recompute** 

