import firebase_admin
from firebase_admin import credentials, db
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

# Firebase
SERVICE_ACCOUNT_PATH = "lifealert-40baf-firebase-adminsdk-fbsvc-d247c8a09d.json"
DATABASE_URL = "https://lifealert-40baf-default-rtdb.firebaseio.com/"

cred = credentials.Certificate(SERVICE_ACCOUNT_PATH)
firebase_admin.initialize_app(cred, {"databaseURL": DATABASE_URL})

WINDOW_SIZE = 50

timestamps = deque(maxlen=WINDOW_SIZE)
heart_rate = deque(maxlen=WINDOW_SIZE)
spo2 = deque(maxlen=WINDOW_SIZE)
temp = deque(maxlen=WINDOW_SIZE)
acc_x = deque(maxlen=WINDOW_SIZE)
acc_y = deque(maxlen=WINDOW_SIZE)
acc_z = deque(maxlen=WINDOW_SIZE)
gyro_x = deque(maxlen=WINDOW_SIZE)
gyro_y = deque(maxlen=WINDOW_SIZE)
gyro_z = deque(maxlen=WINDOW_SIZE)

latest_lat = None
latest_lng = None

fig, axes = plt.subplots(5, 1, figsize=(12, 12), sharex=True)

gps_text = fig.text(0.5, 0.95, "Lat = N/A, Lng = N/A", ha="center", fontsize=14, color="darkred", weight="bold")

def fetch_data():
    global latest_lat, latest_lng
    sensor_ref = db.reference("sensorData")
    data = sensor_ref.get()

    if not data:
        return

    records = []
    for key, value in data.items():
        record = {
            "timestamp": value.get("timestamp", None),  # **确保 timestamp 作为 X 轴**
            "heart_rate": value.get("health", {}).get("heart_rate", None),
            "spo2": value.get("health", {}).get("spo2", None),
            "temp": value.get("health", {}).get("temp", None),
            "acc_x": value.get("imu", {}).get("acceleration", {}).get("x", None),
            "acc_y": value.get("imu", {}).get("acceleration", {}).get("y", None),
            "acc_z": value.get("imu", {}).get("acceleration", {}).get("z", None),
            "gyro_x": value.get("imu", {}).get("gyro", {}).get("x", None),
            "gyro_y": value.get("imu", {}).get("gyro", {}).get("y", None),
            "gyro_z": value.get("imu", {}).get("gyro", {}).get("z", None),
            "lat": value.get("location", {}).get("lat", None),  # ✅ 获取最新经纬度
            "lng": value.get("location", {}).get("lng", None),
        }
        records.append(record)

    df = pd.DataFrame(records)
    df = df.sort_values(by="timestamp")

    df["timestamp"] = pd.to_numeric(df["timestamp"], errors="coerce")

    for col in ["heart_rate", "spo2", "temp", "acc_x", "acc_y", "acc_z", "gyro_x", "gyro_y", "gyro_z"]:
        df[col] = pd.to_numeric(df[col], errors="coerce")

    if not df.empty:
        for i in range(len(df)):
            timestamps.append(df["timestamp"].iloc[i])
            heart_rate.append(df["heart_rate"].iloc[i])
            spo2.append(df["spo2"].iloc[i])
            temp.append(df["temp"].iloc[i])
            acc_x.append(df["acc_x"].iloc[i])
            acc_y.append(df["acc_y"].iloc[i])
            acc_z.append(df["acc_z"].iloc[i])
            gyro_x.append(df["gyro_x"].iloc[i])
            gyro_y.append(df["gyro_y"].iloc[i])
            gyro_z.append(df["gyro_z"].iloc[i])

        latest_lat = df["lat"].dropna().iloc[-1] if not df["lat"].dropna().empty else None
        latest_lng = df["lng"].dropna().iloc[-1] if not df["lng"].dropna().empty else None

def update(frame):
    fetch_data()

    for ax in axes:
        ax.clear()

    x = list(timestamps)

    axes[0].plot(x, list(heart_rate), marker="o", color="b", label="Heart Rate (BPM)")
    axes[0].set_ylabel("Heart Rate (BPM)")
    axes[0].set_title("Heart Rate Over Time")
    axes[0].legend(loc="upper right", frameon=True)

    axes[1].plot(x, list(spo2), marker="s", color="orange", label="SpO2 (%)")
    axes[1].set_ylabel("SpO2 (%)")
    axes[1].set_title("SpO2 Over Time")
    axes[1].legend(loc="upper right", frameon=True)

    axes[2].plot(x, list(temp), marker="^", color="green", label="Temperature (°C)")
    axes[2].set_ylabel("Temperature (°C)")
    axes[2].set_title("Temperature Over Time")
    axes[2].legend(loc="upper right", frameon=True)

    axes[3].plot(x, list(acc_x), linestyle="--", label="Acc X", color="red")
    axes[3].plot(x, list(acc_y), linestyle="--", label="Acc Y", color="blue")
    axes[3].plot(x, list(acc_z), linestyle="--", label="Acc Z", color="green")
    axes[3].set_ylabel("Acceleration (m/s²)")
    axes[3].set_title("Acceleration Over Time")
    axes[3].legend(loc="upper right", frameon=True)

    axes[4].plot(x, list(gyro_x), linestyle="-.", label="Gyro X", color="red")
    axes[4].plot(x, list(gyro_y), linestyle="-.", label="Gyro Y", color="blue")
    axes[4].plot(x, list(gyro_z), linestyle="-.", label="Gyro Z", color="green")
    axes[4].set_xlabel("Timestamp")  # ✅ **确保 timestamp 是 X 轴**
    axes[4].set_ylabel("Gyroscope (rad/s)")
    axes[4].set_title("Gyroscope Data Over Time")
    axes[4].legend(loc="upper right", frameon=True)

    # ensure proper spacing for all plots
    plt.tight_layout()

    if latest_lat is not None and latest_lng is not None:
        gps_text.set_text(f"Geo location: Lat = {latest_lat:.5f}, Lng = {latest_lng:.5f}")
    else:
        gps_text.set_text("Geo location: Lat = N/A, Lng = N/A")

#
ani = animation.FuncAnimation(fig, update, interval=1000, cache_frame_data=False)

plt.show()

