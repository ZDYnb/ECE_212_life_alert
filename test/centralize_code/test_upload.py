import firebase_admin
from firebase_admin import credentials, db
import time

# ✅ 你的 Firebase Admin SDK 密钥文件
SERVICE_ACCOUNT_PATH = "lifealert-40baf-firebase-adminsdk-fbsvc-5bdb920efa.json"
DATABASE_URL = "https://lifealert-40baf-default-rtdb.firebaseio.com/"

# ✅ 初始化 Firebase Admin SDK
cred = credentials.Certificate(SERVICE_ACCOUNT_PATH)
firebase_admin.initialize_app(cred, {"databaseURL": DATABASE_URL})


ref = db.reference("sensorData")
ref.delete()  # 删除整个 sensorData 节点
print("sensorData deleted")