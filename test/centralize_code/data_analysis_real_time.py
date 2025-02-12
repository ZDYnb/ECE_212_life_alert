import firebase_admin
from firebase_admin import credentials, db
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import requests
from collections import deque

# Firebase 配置
SERVICE_ACCOUNT_PATH = "lifealert-40baf-firebase-adminsdk-fbsvc-d247c8a09d.json"
DATABASE_URL = "https://lifealert-40baf-default-rtdb.firebaseio.com/"

cred = credentials.Certificate(SERVICE_ACCOUNT_PATH)
firebase_admin.initialize_app(cred, {"databaseURL": DATABASE_URL})

# Discord Webhook
DISCORD_WEBHOOK_URL = "https://discord.com/api/webhooks/1336791999172313138/IbW-LUezfaqIEoihgNqRmwG0kwA_3l3jMvYvDJc_o5LBkWjqhfKxg8VaXSjksHj2Qfxw"

# 窗口大小
WINDOW_SIZE = 50

# 数据存储
timestamps = deque(maxlen=WINDOW_SIZE)
heart_rate = deque(maxlen=WINDOW_SIZE)
avg_bpm = deque(maxlen=WINDOW_SIZE)
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


def send_discord_alert(data):
    """ 发送紧急通知到 Discord 机器人 """
    message = {
        "username": "LifeAlertBot",
        "avatar_url": "https://cdn-icons-png.flaticon.com/512/564/564619.png",
        "embeds": [{
            "title": "🚨 **Emergency Alert!**",
            "description": f"💓 **Heart Rate:** {data['heart_rate']} bpm\n"
                           f"📊 **Avg BPM:** {data['avg_bpm']} bpm\n"
                           f"🌡 **Temperature:** {data['temperature']}°C\n"
                           f"📍 **Location:** ({data['lat']}, {data['lng']})",
            "color": 16711680
        }]
    }

    response = requests.post(DISCORD_WEBHOOK_URL, json=message, headers={"Content-Type": "application/json"})
    if response.status_code == 204:
        print("✅ Emergency alert sent to Discord!")
    else:
        print("❌ Failed to send alert:", response.text)


def fetch_data():
    """ 获取 Firebase 数据 """
    global latest_lat, latest_lng
    sensor_ref = db.reference("sensorData")
    data = sensor_ref.get()

    if not data:
        return

    records = []
    for key, value in data.items():
        record = {
            "timestamp": value.get("timestamp", None),
            "heart_rate": value.get("heart_rate", None),
            "avg_bpm": value.get("avg_bpm", None),
            "temperature": value.get("temperature", None),
            "acc_x": value.get("imu", {}).get("ax", None),
            "acc_y": value.get("imu", {}).get("ay", None),
            "acc_z": value.get("imu", {}).get("az", None),
            "gyro_x": value.get("imu", {}).get("gx", None),
            "gyro_y": value.get("imu", {}).get("gy", None),
            "gyro_z": value.get("imu", {}).get("gz", None),
            "lat": value.get("location", {}).get("lat", None),
            "lng": value.get("location", {}).get("lng", None),
            "emergency": value.get("emergency", False)
        }
        records.append(record)

    df = pd.DataFrame(records)
    df = df.sort_values(by="timestamp")
    df["timestamp"] = pd.to_numeric(df["timestamp"], errors="coerce")

    for col in ["heart_rate", "avg_bpm", "temperature", "acc_x", "acc_y", "acc_z", "gyro_x", "gyro_y", "gyro_z"]:
        df[col] = pd.to_numeric(df[col], errors="coerce")

    if not df.empty:
        for i in range(len(df)):
            timestamps.append(df["timestamp"].iloc[i])
            heart_rate.append(df["heart_rate"].iloc[i])
            avg_bpm.append(df["avg_bpm"].iloc[i])
            temp.append(df["temperature"].iloc[i])
            acc_x.append(df["acc_x"].iloc[i])
            acc_y.append(df["acc_y"].iloc[i])
            acc_z.append(df["acc_z"].iloc[i])
            gyro_x.append(df["gyro_x"].iloc[i])
            gyro_y.append(df["gyro_y"].iloc[i])
            gyro_z.append(df["gyro_z"].iloc[i])

            if df["emergency"].iloc[i]:  # ✅ 触发紧急警报
                send_discord_alert(df.iloc[i].to_dict())

        latest_lat = df["lat"].dropna().iloc[-1] if not df["lat"].dropna().empty else None
        latest_lng = df["lng"].dropna().iloc[-1] if not df["lng"].dropna().empty else None


def update(frame):
    """ 实时更新数据 """
    fetch_data()

    for ax in axes:
        ax.clear()

    x = list(timestamps)

    axes[0].plot(x, list(heart_rate), marker="o", color="b", label="Heart Rate (BPM)")
    axes[0].set_ylabel("Heart Rate (BPM)")
    axes[0].set_title("Heart Rate Over Time")
    axes[0].legend(loc="upper right", frameon=True)

    axes[1].plot(x, list(avg_bpm), marker="s", color="purple", label="Avg BPM")
    axes[1].set_ylabel("Avg BPM")
    axes[1].set_title("Avg BPM Over Time")
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
    axes[4].set_xlabel("Timestamp")
    axes[4].set_ylabel("Gyroscope (rad/s)")
    axes[4].set_title("Gyroscope Data Over Time")
    axes[4].legend(loc="upper right", frameon=True)

    plt.tight_layout()

    if latest_lat is not None and latest_lng is not None:
        gps_text.set_text(f"Geo location: Lat = {latest_lat:.5f}, Lng = {latest_lng:.5f}")
    else:
        gps_text.set_text("Geo location: Lat = N/A, Lng = N/A")


ani = animation.FuncAnimation(fig, update, interval=1000, cache_frame_data=False)
plt.show()
