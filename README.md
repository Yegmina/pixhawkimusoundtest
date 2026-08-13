# Pixhawk IMU sound monitor

A local browser dashboard for a Pixhawk running MAVLink/PX4. It plots the primary IMU and compass, reserves a separate chart for an external CAN magnetometer (such as an RM3100), and can turn movement into an adjustable browser tone.

## Run on Linux

Connect the Pixhawk by USB, then clone and run:

```bash
git clone https://github.com/Yegmina/pixhawkimusoundtest.git
cd pixhawkimusoundtest
python3 -m venv .venv
. .venv/bin/activate
pip install -r requirements.txt
python app.py
```

Open http://localhost:8000, then click **Enable sound** once to allow the browser to play audio.

The default MAVLink link is `/dev/ttyACM0` at 115200 baud. To choose another link or port:

```bash
PIXHAWK_PORT=/dev/ttyACM1 PIXHAWK_BAUD=115200 PIXHAWK_STREAM_HZ=20 PORT=8000 python app.py
```

If the serial port is permission denied, add the current user to `dialout`, log out/in, and rerun:

```bash
sudo usermod -aG dialout "$USER"
```

## Dashboard controls

- Auto or manual graph scale and 10/30/60-second history.
- Primary IMU and compass from `RAW_IMU` (avoids duplicate MAVLink streams).
- External magnetometer view from `SCALED_IMU2/3`.
- Tone frequency, maximum volume, gyroscope movement limit, and start-motion mute time.
- Orange dashed lines in the gyroscope graph show the active sound threshold.

The server only requests temporary MAVLink telemetry intervals. It does not change Pixhawk parameters or issue flight commands.
