## Trajectory generation with RNB-PLANNNING
* Compatible with rnb-planning branch feature-traj-for-control commit on 2021.6.25

1. Prepare the planner
    * Go to the planner computer and turn it on (ip: 192.168.0.123 in local network)
    * Open a terminal, start jupyter. Don't close the terminal.
    ```bash
    cd ~ && jupyter notebook
    ```
    * A brower will open jupyter page. If not, open one. (http://192.168.0.123:8888/)

2. Run the planner
    * Open the planner script *~/Projects/rnb-planning/release/7.Demo/7.1.Sweeping/7.1.2.WhiteBoardSweeping-one-by-one.ipynb*
    * Put "track" on the workspace, as shown in the figure on the top of the *7.1.2.WhiteBoardSweeping-one-by-one.ipynb* file.
    * Run the script cell-by-cell.
        * If you proceed down to **add sweep face**, you will see the environment and "track" is added on the RVIZ.
        * Run down to **play schedule**. You will see the motion in the RVIZ.
        * Run **Save test trajectory**
    * **[IMPORTANT]** Don't just close or restart the notebook. The ON/OFF state of RVIZ needs to be synchronized.
        1. Click the stop button
        2. Check the RVIZ is closed
        3. Restart the kernel.
        
3. Execute the trajectory (This can be done from another computer. Access to the planner computer at http://192.168.0.123:8888/)
    * Open control test script *~/Projects/rnb-planning/src/scripts/[TEST]RNB-CONTROL-Indy-Motion.ipynb*
    * Run the sweep motion for each **LINE_NO**

4. (If you want) Do the same with *7.1.3.WhiteBoardSweeping-curved.ipynb* to test with the curved surface.

**[NOTE]** To change the tool offset, change tool_offset value when you call add_indy_sweep_tool().  

