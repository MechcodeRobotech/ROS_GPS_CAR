# GPS_Localize Viewer — โปรแกรมดูหน้าเว็บ (ไม่ต้องมี ROS)

โปรแกรมนี้เปิด **หน้าเว็บเดียวกับของหุ่นยนต์** บนคอมพิวเตอร์ธรรมดา
ใช้เพื่อดูหน้าตาโปรแกรม ลองกดดู และ **แก้ไฟล์ตั้งค่า/จุดหมายล่วงหน้า**

> ⚠️ **โหมดสาธิต — ไม่ได้ต่อกับหุ่นยนต์จริง**
> ปุ่มทุกปุ่มกดได้ แต่สั่งได้แค่ **หุ่นจำลอง** บนแผนที่เท่านั้น
> โปรแกรมนี้ไม่มี ROS และไม่มีวิทยุ จึงไปถึงหุ่นจริงไม่ได้เลย
> ตัวเลขที่เห็นเป็น **ข้อมูลจำลอง** ทั้งหมด

สาธิตได้ครบ: ขับด้วยแป้นพิมพ์ · หมุนตามองศา / หันไปทิศ N-E-S-W ·
คลิกแผนที่แล้ว “ไปที่นี่เลย” · ลากลูกศรตั้งตำแหน่งเริ่มต้น · หยุดฉุกเฉิน ·
เพิ่ม/แก้จุดหมาย · แก้ค่าตั้งทุกกลุ่ม

## Windows

1. คัดลอกโฟลเดอร์ `webapp` ทั้งโฟลเดอร์ไปเครื่องที่ต้องการ
2. ดับเบิลคลิก **`run_windows.bat`** (หรือ `exe\GPS_Localize_Viewer.exe` โดยตรง)
3. เบราว์เซอร์จะเปิด `http://localhost:8080` ให้เอง

ไฟล์ `.exe` ไม่ได้อยู่ใน Git: สร้างด้วย `build_exe.bat` หรือใช้ Python 3 + PyYAML (`python -m pip install pyyaml`) แล้วเปิด `run_windows.bat`

สร้าง `.exe` ใหม่เอง (เมื่อแก้โค้ด): ดับเบิลคลิก `build_exe.bat` (ต้องมี Python 3 + pip)

## Ubuntu / Linux

```bash
cd webapp
chmod +x run_ubuntu.sh
./run_ubuntu.sh
```

ใช้ `python3` ที่มากับ Ubuntu ได้เลย (ต้องมี `python3-yaml`:
`sudo apt install python3-yaml`) หรือจะสร้างไฟล์ไบนารีเดี่ยวด้วย `./build_linux.sh`
ก็ได้ (ต้อง build บน Linux เท่านั้น)

## ตัวเลือกตอนเปิด / Command line options

| Option | ทำอะไร |
|---|---|
| `--host 0.0.0.0` | ให้เครื่องอื่นในวง Wi-Fi เปิดดู *เดโม* นี้ได้ (ยังเป็นหุ่นจำลอง) |
| `--port 8081` | เปลี่ยนพอร์ต (ค่าเริ่มต้น 8080) |
| `--config-dir D:\my_settings` | เลือกโฟลเดอร์เก็บไฟล์ตั้งค่า |
| `--robot-url http://192.168.1.50:8080` | ใส่ที่อยู่หุ่นยนต์ไว้ล่วงหน้า มีปุ่มลัดให้กดข้ามไป |
| `--no-demo` | ปิดหุ่นจำลอง แสดงค่าว่าง และล็อกปุ่มควบคุมทั้งหมด |
| `--no-browser` | ไม่ต้องเปิดเบราว์เซอร์ให้อัตโนมัติ |

ตัวอย่าง:

```bat
run_windows.bat --robot-url http://192.168.1.50:8080
```

## แก้ไฟล์ตั้งค่าล่วงหน้าแล้วเอาไปใช้กับหุ่นจริง

1. เปิด Viewer → หน้า **ตั้งค่า** และ **จุดหมาย** → แก้ → กด **บันทึก**
2. ไฟล์จะถูกเขียนที่โฟลเดอร์ `settings\` ข้าง ๆ โปรแกรม
   (เช่น `webapp\exe\settings\waypoints.yaml`)
3. คัดลอกไฟล์ไปที่เครื่องหุ่นยนต์ที่ `~/.gps_localize/`

```bash
scp settings/waypoints.yaml  ubuntu@192.168.1.50:~/.gps_localize/
scp settings/nav.yaml        ubuntu@192.168.1.50:~/.gps_localize/
```

4. บนหน้าเว็บของหุ่นจริง กด **โหลดใหม่ / RELOAD** หรือรีสตาร์ตโปรแกรม

## ควบคุมหุ่นจริงต้องทำยังไง

เครื่อง Ubuntu ที่ต่อกับหุ่น (server PC) รันคำสั่งเดียว:

```bash
ros2 launch gps_localize bringup.launch.py
```

จากนั้นมือถือ แท็บเล็ต หรือคอมเครื่องไหนก็ได้ที่อยู่ Wi-Fi เดียวกัน
เปิดเบราว์เซอร์ไปที่ `http://<ไอพีของ server PC>:8080` แล้วคุมได้เต็มรูปแบบ
รายละเอียด: [../docs/web_ui.md](../docs/web_ui.md)

## Run a real server on Windows

This viewer is a demonstration. For a real robot server, follow the
[Docker guide](../docs/docker.md): Docker Desktop for Wi-Fi, or WSL2 with a
local Docker Engine for USB. Open that server URL in your browser. Do not run
the viewer on port 8080 at the same time as the server.
