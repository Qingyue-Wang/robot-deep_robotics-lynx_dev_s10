#!/bin/bash
# 依次走 4 个观察位各拍一张，并拼成 2x2 拼图
set -e
cd /home/dog/robot-deep_robotics-lynx_s10
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=43
/usr/bin/python3 -u scripts/pose_tour.py > /tmp/pose_tour_d.log 2>&1
cd logs/pose_shots
/usr/bin/python3 - <<'EOF'
import cv2
names = [("1_nominal", "1  nominal"), ("2_rec1", "2  rec1"), ("3_rec2", "3  rec2"), ("4_home", "4  home")]
imgs = []
for f, label in names:
    im = cv2.resize(cv2.imread(f + ".jpg"), (640, 360))
    cv2.rectangle(im, (0, 0), (210, 34), (0, 0, 0), -1)
    cv2.putText(im, label, (8, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    imgs.append(im)
sheet = cv2.vconcat([cv2.hconcat(imgs[:2]), cv2.hconcat(imgs[2:])])
cv2.imwrite("tour_4views_d.jpg", sheet, [cv2.IMWRITE_JPEG_QUALITY, 90])
print("montage ok", sheet.shape, flush=True)
EOF
echo done
