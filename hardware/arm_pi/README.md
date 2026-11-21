# Arm control scripts (Raspberry Pi 4 + PCA9685)

Standalone scripts used to calibrate, teach and replay the 5-joint inspection arm.
These are separate from the ROS 2 `arm_control` package.

| Script | Purpose |
| --- | --- |
| `calibrate.py` | Type a PCA9685 channel and angle to move one servo |
| `server.py` | Web sliders and in-memory record/replay (first version) |
| `record_server.py` | Same, with smoothed motion and Save to `sequence.json` |
| `run_task.py` | Go home, then loop the saved `sequence.json` until Ctrl+C |

Channels: A base = 0, B shoulder = 1 and 5 (mirrored, 180 - angle), C elbow = 2, D wrist = 3, E gripper = 4.

## Run

```bash
pip install -r requirements.txt
python3 calibrate.py        # find safe angle limits first
python3 record_server.py    # open http://<pi-ip>:5000, record, press SAVE
python3 run_task.py         # replays sequence.json in a loop
```

## Safety

- `run_task.py` loops forever. Clear hands and the workspace before starting.
- The web servers listen on 0.0.0.0 with CORS open and no login. Use a trusted lab network only.
- Check joint limits with `calibrate.py` before recording. Sliders in `server.py` allow 0-180 degrees.
