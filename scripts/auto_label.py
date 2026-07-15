#!/usr/bin/env python3
"""HSV 颜色分割自动标注绿色方块 → YOLO OBB 格式"""
import os, sys, math, cv2
import numpy as np


def auto_label(img_path: str, out_dir: str):
    """对一张图做绿色分割 + minAreaRect，输出 YOLO OBB txt"""
    img = cv2.imread(img_path)
    if img is None:
        return None

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # 绿色阈值（HSV 上下界，可调）
    lower_green = np.array([35, 60, 60])
    upper_green = np.array([85, 255, 255])
    mask = cv2.inRange(hsv, lower_green, upper_green)

    # 形态学去噪
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    # 取最大轮廓
    cnt = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(cnt)
    if area < 50:  # 太小，噪声
        return None

    rect = cv2.minAreaRect(cnt)
    # rect 四个角点 (float32)，顺序: 左下→左上→右上→右下（或顺时针）
    box = cv2.boxPoints(rect)  # shape (4, 2)

    H, W = img.shape[:2]

    # YOLO OBB 格式 (ultralytics >= 8.1.0):
    # class_id x1 y1 x2 y2 x3 y3 x4 y4 （四点归一化坐标）
    vals = []
    for pt in box:
        vals.append(f'{pt[0]/W:.6f}')
        vals.append(f'{pt[1]/H:.6f}')

    base = os.path.splitext(os.path.basename(img_path))[0]
    out_path = os.path.join(out_dir, f'{base}.txt')
    with open(out_path, 'w') as f:
        f.write(f'0 {" ".join(vals)}\n')

    return area


def main():
    img_dir = sys.argv[1] if len(sys.argv) > 1 else '/tmp/train_data'
    out_dir = os.path.join(img_dir, 'labels')
    os.makedirs(out_dir, exist_ok=True)

    imgs = sorted([f for f in os.listdir(img_dir) if f.endswith('.jpg')])
    if not imgs:
        print(f'在 {img_dir} 里没找到 jpg 图片'); return

    ok, fail = 0, 0
    for name in imgs:
        area = auto_label(os.path.join(img_dir, name), out_dir)
        if area:
            ok += 1
        else:
            fail += 1
            print(f'  ✗ {name} — 未检测到绿色区域')

    print(f'\n完成: {ok} 张已标注, {fail} 张失败 → {out_dir}')


if __name__ == '__main__':
    main()
