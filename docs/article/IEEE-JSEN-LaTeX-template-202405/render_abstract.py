#!/usr/bin/env python3
"""
Render SPA-KLT Graphical Abstract as a high-resolution PNG.
Canvas: 1920 x 1080 (16:9)
"""

from PIL import Image, ImageDraw, ImageFont
import os

# Canvas dimensions
W, H = 1920, 1080
BG_COLOR = (13, 27, 42)          # #0D1B2A
HEADER_COLOR = (0, 180, 216)      # #00B4D8
FOOTER_COLOR = (0, 180, 216)
S1_COLOR = (255, 107, 107)        # #FF6B6B (coral)
S2_COLOR = (0, 180, 216)          # #00B4D8
S3_COLOR = (46, 204, 113)         # #2ECC71
S4_COLOR = (243, 156, 18)         # #F39C12
WHITE = (255, 255, 255)
GRAY = (180, 180, 180)
DARK_GRAY = (100, 100, 100)
LIGHT_BLUE = (144, 224, 239)
GREEN_DARK = (20, 90, 50)         # #145A32
BLUE_DARK = (10, 61, 98)          # #0A3D62
NAVY_LIGHT = (27, 58, 75)         # #1B3A4B
RED_DARK = (123, 36, 28)          # #7B241C
ORANGE_DARK = (125, 118, 8)       # #7D6608
BLUE_MID = (26, 82, 118)          # #1A5276
GREEN_MID2 = (15, 120, 70)
TEXT_BG3 = (0, 119, 182)          # #0077B6

# Font helpers
FONT_TITLE = None
FONT_H1 = None
FONT_H2 = None
FONT_BODY = None
FONT_MONO = None
FONT_BIG = None
FONT_BIG2 = None

def load_fonts():
    global FONT_TITLE, FONT_H1, FONT_H2, FONT_BODY, FONT_MONO, FONT_BIG, FONT_BIG2
    font_paths = [
        "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
        "/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
        "/usr/share/fonts/truetype/freefont/FreeSans.ttf",
        "/usr/share/fonts/truetype/freefont/FreeSansBold.ttf",
    ]
    bold_paths = [p for p in font_paths if "Bold" in p]
    reg_paths  = [p for p in font_paths if "Bold" not in p]

    def try_font(paths, sizes):
        for sz in sizes:
            for p in paths:
                try:
                    return ImageFont.truetype(p, sz)
                except:
                    pass
        return ImageFont.load_default()

    FONT_TITLE = try_font(bold_paths, [26, 24, 22, 20])
    FONT_H1    = try_font(bold_paths, [19, 17, 15])
    FONT_H2    = try_font(bold_paths, [15, 13, 11])
    FONT_BODY  = try_font(reg_paths,  [12, 11, 10])
    FONT_MONO  = try_font([p for p in font_paths if "mono" in p.lower() or "courier" in p.lower()], [11, 10, 9])
    if FONT_MONO == ImageFont.load_default():
        FONT_MONO = try_font(reg_paths, [11, 10, 9])
    FONT_BIG   = try_font(bold_paths, [54, 48, 42, 36])
    FONT_BIG2  = try_font(bold_paths, [38, 34, 30])
    FONT_SMALL = try_font(reg_paths, [10, 9, 8])
    FONT_FTR   = try_font(reg_paths, [11, 10, 9])
    return FONT_SMALL, FONT_FTR

FONT_SMALL = None
FONT_FTR = None


def rounded_rect(draw, xy, radius, fill, outline=None, width=1):
    x0, y0, x1, y1 = xy
    r = radius
    draw.rectangle([x0+r, y0, x1-r, y1], fill=fill, outline=outline, width=width)
    draw.rectangle([x0, y0+r, x1, y1-r], fill=fill, outline=outline, width=width)
    draw.pieslice([x0, y0, x0+2*r, y0+2*r], 180, 270, fill=fill, outline=outline, width=width)
    draw.pieslice([x1-2*r, y0, x1, y0+2*r], 270, 360, fill=fill, outline=outline, width=width)
    draw.pieslice([x0, y1-2*r, x0+2*r, y1], 90, 180, fill=fill, outline=outline, width=width)
    draw.pieslice([x1-2*r, y1-2*r, x1, y1], 0, 90, fill=fill, outline=outline, width=width)


def draw_text_centered(draw, text, cx, y, font, color):
    bbox = draw.textbbox((0,0), text, font=font)
    w = bbox[2] - bbox[0]
    draw.text((cx - w//2, y), text, font=font, fill=color)


def draw_multiline(draw, lines, x, y, font, color, line_h=None, align="left"):
    if line_h is None:
        bbox = draw.textbbox((0,0), "Ay", font=font)
        line_h = bbox[3] - bbox[1] + 3
    for i, line in enumerate(lines):
        if align == "center":
            bbox = draw.textbbox((0,0), line, font=font)
            w = bbox[2] - bbox[0]
            draw.text((x - w//2, y + i*line_h), line, font=font, fill=color)
        else:
            draw.text((x, y + i*line_h), line, font=font, fill=color)


def section_divider(draw, x, color, total_h=970, y_start=80):
    draw.rectangle([x, y_start, x+7, y_start+total_h], fill=color)


def section_header(draw, x, w, y, h, color, text, font):
    draw.rectangle([x, y, x+w, y+h], fill=(color[0]//5, color[1]//5, color[2]//5))
    th = draw.textbbox((0,0), "Ay", font=font)[3] - draw.textbbox((0,0), "Ay", font=font)[1]
    ty = y + (h - th) // 2
    draw_text_centered(draw, text, x + w//2, ty, font, color)


def card(draw, x, y, w, h, fill, stroke, stroke_w=2, radius=6):
    rounded_rect(draw, (x, y, x+w, y+h), radius, fill)
    if stroke:
        rounded_rect(draw, (x, y, x+w, y+h), radius, None, stroke, stroke_w)


def arrow(draw, x0, y0, x1, y1, color, w=3):
    draw.line([(x0, y0), (x1, y1)], fill=color, width=w)
    import math
    dx, dy = x1-x0, y1-y0
    length = math.sqrt(dx*dx+dy*dy)
    if length == 0:
        return
    dx, dy = dx/length, dy/length
    aw = 12
    ax1 = x1 - dx*aw - dy*aw*0.4
    ay1 = y1 - dy*aw + dx*aw*0.4
    ax2 = x1 - dx*aw + dy*aw*0.4
    ay2 = y1 - dy*aw - dx*aw*0.4
    draw.polygon([(x1,y1),(ax1,ay1),(ax2,ay2)], fill=color)


def chip(draw, x, y, w, h, text, font, fg, bg, radius=4):
    rounded_rect(draw, (x, y, x+w, y+h), radius, bg)
    th = draw.textbbox((0,0),"Ay",font=font)[3]-draw.textbbox((0,0),"Ay",font=font)[1]
    tx = x + w//2 - draw.textbbox((0,0),text,font=font)[2]//2
    ty = y+(h-th)//2
    draw.text((tx, ty), text, font=font, fill=fg)


def render():
    global FONT_SMALL, FONT_FTR
    FONT_SMALL, FONT_FTR = load_fonts()

    img = Image.new("RGB", (W, H), BG_COLOR)
    draw = ImageDraw.Draw(img)

    # ---- Header ----
    draw.rectangle([0, 0, W, 78], fill=HEADER_COLOR)
    draw_text_centered(draw,
        "SPA-KLT: Adaptive Sparse Optical Flow Tracking for Visual-Inertial Odometry",
        W//2, 20, FONT_TITLE, WHITE)

    # ---- Footer ----
    draw.rectangle([0, 1050, W, H], fill=FOOTER_COLOR)
    draw_text_centered(draw, "IEEE Sensors Journal  |  Graphical Abstract  |  2026",
                        W//2, 1054, FONT_FTR, WHITE)

    # Section boundaries
    S1_X0, S1_X1 = 0, 467
    S2_X0, S2_X1 = 467, 1175
    S3_X0, S3_X1 = 1175, 1523
    S4_X0, S4_X1 = 1523, 1920
    MAIN_Y0, MAIN_Y1 = 80, 1050

    # ---- Dividers ----
    section_divider(draw, S1_X1-7, S1_COLOR)
    section_divider(draw, S2_X1-7, S2_COLOR)
    section_divider(draw, S3_X1-7, S3_COLOR)

    # Section vertical labels
    def vlabel(draw, text, x, cy, font, color):
        bbox = draw.textbbox((0,0), text, font=font)
        th = bbox[3]-bbox[1]
        tw = bbox[2]-bbox[0]
        import math
        img2 = img.copy()
        rotated = img2.rotate(90, expand=True)
        draw2 = ImageDraw.Draw(rotated)
        rx = cy - tw//2
        ry = (W - x - th//2)
        draw2.text((rx, ry), text, font=font, fill=color)
        import numpy as np
        arr = np.array(rotated)
        angle = 270
        from PIL import Image as PILImage
        result = PILImage.fromarray(arr).rotate(angle, expand=True)
        draw3 = ImageDraw.Draw(result)
        # just paste it back
        ox = x - (result.width - (S2_X1-S2_X0))//2
        oy = int(cy - result.height/2)
        img.paste(result, (ox, oy), result.split()[3] if result.mode == 'RGBA' else None)

    # ---- Section Headers ----
    section_header(draw, S1_X0+8, S1_X1-S1_X0-15, MAIN_Y0, 52, S1_COLOR,
                   "Pain Points of Existing Methods", FONT_H1)
    section_header(draw, S2_X0+8, S2_X1-S2_X0-15, MAIN_Y0, 52, S2_COLOR,
                   "SPA-KLT: Conditional Enhancement Layer", FONT_H1)
    section_header(draw, S3_X0+8, S3_X1-S3_X0-15, MAIN_Y0, 52, S3_COLOR,
                   "Experimental Results (EuRoC)", FONT_H1)
    section_header(draw, S4_X0+8, S4_X1-S4_X0-15, MAIN_Y0, 52, S4_COLOR,
                   "Applicable Scenarios & Value", FONT_H1)

    # ===========================
    # SECTION 1: Pain Points
    # ===========================
    s1cx = (S1_X0 + S1_X1) // 2
    y = 145

    def pain_card(idx, icon, title, desc_lines, y):
        card_w = S1_X1 - S1_X0 - 30
        card(draw, S1_X0+15, y, card_w, 88, NAVY_LIGHT, S1_COLOR, 2, 6)
        draw.text((S1_X0+22, y+6), icon, font=FONT_H2, fill=S1_COLOR)
        draw.text((S1_X0+60, y+4), title, font=FONT_H2, fill=S1_COLOR)
        draw_multiline(draw, desc_lines, S1_X0+60, y+32, FONT_SMALL, GRAY)
        return 88 + 10

    dy = pain_card(1, "[FIXED]", "Fixed KLT Parameters",
                    ["  Fixed search window & pyramid levels",
                     "  -> Wasted compute on small displacement",
                     "  -> KLT is the frontend bottleneck"], y)
    y += dy
    dy = pain_card(2, "[FAST]", "Fast Motion & Motion Blur",
                    ["  High-speed rotation degrades KLT tracking",
                     "  -> Feature loss -> Aggressive re-detection",
                     "  -> Vicious cycle of compute & drift"], y)
    y += dy
    dy = pain_card(3, "[LOOP]", "Loop Closure False Positives",
                    ["  DBoW2 alone: no motion-consistency filter",
                     "  -> Repeated textures cause false loops",
                     "  -> Pollutes global pose graph"], y)
    y += dy
    dy = pain_card(4, "[SPA!]", "SPA Assumptions Fail",
                    ["  Patch overlap, unit-depth, brightness,",
                     "  strong texture: ALL must hold -> rarely!",
                     "  -> Without fallback, hurts frontend"], y)
    y += dy

    # ===========================
    # SECTION 2: Algorithm Framework
    # ===========================
    s2cx = (S2_X0 + S2_X1) // 2

    # --- IMU Box ---
    imu_x, imu_y, imu_w, imu_h = S2_X0+30, 155, 165, 72
    card(draw, imu_x, imu_y, imu_w, imu_h, BLUE_DARK, S2_COLOR, 2, 8)
    draw.text((imu_x+8, imu_y+4), "IMU PREINTEGRATION", font=FONT_SMALL, fill=S2_COLOR)
    draw.text((imu_x+8, imu_y+26), "R_imu = rotation", font=FONT_SMALL, fill=GRAY)
    draw.text((imu_x+8, imu_y+44), "between frames", font=FONT_SMALL, fill=GRAY)

    # Arrow: IMU -> SPA
    ax0 = imu_x + imu_w; ay = imu_y + imu_h//2
    ax1 = S2_X0+265;    # SPA left
    arrow(draw, ax0, ay, ax1, ay, S2_COLOR, 3)

    # --- SPA Box ---
    spa_x, spa_y, spa_w, spa_h = S2_X0+265, 143, 175, 96
    card(draw, spa_x, spa_y, spa_w, spa_h, BLUE_DARK, LIGHT_BLUE, 3, 8)
    draw.text((spa_x+spa_w//2 - draw.textbbox((0,0),"Sparse Photometric Alignment",font=FONT_SMALL)[2]//2,
               spa_y+6), "Sparse Photometric Alignment", font=FONT_SMALL, fill=LIGHT_BLUE)
    draw.text((spa_x+8, spa_y+26), "(SPA)", font=FONT_SMALL, fill=LIGHT_BLUE)
    draw_multiline(draw, ["Gauss-Newton on SO(3)",
                          "Iterative: max 4 steps",
                          "Output: chi2_final, R_sparse"],
                  spa_x+8, spa_y+50, FONT_SMALL, GRAY)

    # Arrow: SPA -> Chi2 Gate
    arrow(draw, spa_x+spa_w, spa_y+spa_h//2, S2_X0+450, spa_y+spa_h//2, LIGHT_BLUE, 3)

    # --- Chi2 Adaptive Gate ---
    chi_x, chi_y, chi_w, chi_h = S2_X0+450, 140, 175, 102
    card(draw, chi_x, chi_y, chi_w, chi_h, TEXT_BG3, LIGHT_BLUE, 2, 8)
    draw.text((chi_x+chi_w//2 - draw.textbbox((0,0),"chi2 Adaptive Gate",font=FONT_SMALL)[2]//2,
               chi_y+4), "chi2 Adaptive Gate", font=FONT_SMALL, fill=LIGHT_BLUE)
    tiers = [
        ("chi2 < 5    -> 5x5,  L=1  (save 98.1%)", (46,204,113)),
        ("5<=chi2<15  -> 9x9,  L=1  (save 93.9%)", (243,156,18)),
        ("15<=chi2<50 -> 15x15,L=2 (save 66.0%)", (230,126,34)),
        ("chi2>=50    -> 21x21,L=3 (default)",     (231,76,60)),
    ]
    for i, (txt, tc) in enumerate(tiers):
        ty = chi_y + 28 + i*19
        draw.rectangle([chi_x+6, ty, chi_x+chi_w-6, ty+17], fill=(tc[0]//8, tc[1]//8, tc[2]//8))
        draw.text((chi_x+8, ty+2), txt, font=FONT_SMALL, fill=tc)

    # Arrow: Chi2 -> KLT
    arrow(draw, chi_x+chi_w//2, chi_y+chi_h, chi_x+chi_w//2, S2_X0+260, LIGHT_BLUE, 3)

    # --- Adaptive KLT ---
    klt_x, klt_y, klt_w, klt_h = S2_X0+450, 255, 175, 82
    card(draw, klt_x, klt_y, klt_w, klt_h, BLUE_DARK, LIGHT_BLUE, 2, 8)
    draw.text((klt_x+klt_w//2 - draw.textbbox((0,0),"Adaptive KLT Tracker",font=FONT_SMALL)[2]//2,
               klt_y+6), "Adaptive KLT Tracker", font=FONT_SMALL, fill=LIGHT_BLUE)
    draw_multiline(draw, ["Window + Pyramid levels",
                          "selected by chi2_final",
                          "Fallback: 21x21, L=3"],
                  klt_x+8, klt_y+32, FONT_SMALL, GRAY)

    # Arrow: KLT -> RANSAC/Throttle
    arrow(draw, klt_x+klt_w//2, klt_y+klt_h, klt_x+klt_w//2, 348, LIGHT_BLUE, 2)

    # --- RANSAC + Throttle ---
    thr_x, thr_y, thr_w, thr_h = S2_X0+438, 348, 198, 130
    card(draw, thr_x, thr_y, thr_w, thr_h, BLUE_MID, LIGHT_BLUE, 2, 8)
    draw.text((thr_x+thr_w//2 - draw.textbbox((0,0),"Sparse-Only Frontend Throttling",font=FONT_SMALL)[2]//2,
               thr_y+4), "Sparse-Only Frontend Throttling", font=FONT_SMALL, fill=LIGHT_BLUE)
    items = [
        ("1) Keypoint: every 3 frames", "(0.60ms -> 0.20ms/frame)"),
        ("2) RANSAC conf: 0.99 -> 0.95", "(-40% iterations)"),
    ]
    for i, (a, b) in enumerate(items):
        ty = thr_y + 28 + i*38
        draw.text((thr_x+8, ty), a, font=FONT_SMALL, fill=WHITE)
        draw.text((thr_x+8, ty+16), b, font=FONT_SMALL, fill=(46,204,113))
    chip(draw, thr_x+6, thr_y+thr_h-26, thr_w-12, 20,
         "Condition: used_sparse == TRUE", FONT_SMALL, LIGHT_BLUE, (S2_COLOR[0]//6, S2_COLOR[1]//6, S2_COLOR[2]//6))

    # Arrow: Throttle -> VINS label
    arrow(draw, thr_x+thr_w, thr_y+thr_h//2, S2_X1-30, thr_y+thr_h//2, LIGHT_BLUE, 2)
    draw.text((thr_x+thr_w+4, thr_y+thr_h//2-20), "VINS-Mono", font=FONT_SMALL, fill=GRAY)
    draw.text((thr_x+thr_w+4, thr_y+thr_h//2+2), "(unchanged backend)", font=FONT_SMALL, fill=DARK_GRAY)

    # --- Sparse Rotation Loop Gate ---
    lg_x, lg_y, lg_w, lg_h = S2_X0+265, 258, 175, 80
    card(draw, lg_x, lg_y, lg_w, lg_h, BLUE_MID, LIGHT_BLUE, 2, 8)
    draw.text((lg_x+lg_w//2 - draw.textbbox((0,0),"Sparse Rotation Loop Gate",font=FONT_SMALL)[2]//2,
               lg_y+4), "Sparse Rotation Loop Gate", font=FONT_SMALL, fill=LIGHT_BLUE)
    draw_multiline(draw, ["|theta_sparse - theta_bow| < theta_th",
                          "Reject before reprojection",
                          "theta_th = 3 deg (offline)"],
                  lg_x+8, lg_y+30, FONT_SMALL, GRAY)

    # Arrow: SPA -> Loop Gate (dashed, downwards)
    # Draw a vertical line from SPA down to loop gate
    lx0 = spa_x + spa_w//2; ly0 = spa_y + spa_h
    lx1 = lg_x + lg_w//2; ly1 = lg_y
    # Manually draw dashed arrow
    import math
    segments = 8
    for i in range(segments):
        t0 = i / segments
        t1 = (i + 0.5) / segments
        px0 = lx0 + (lx1-lx0)*t0
        py0 = ly0 + (ly1-ly0)*t0
        px1 = lx0 + (lx1-lx0)*t1
        py1 = ly0 + (ly1-ly0)*t1
        draw.line([(int(px0), int(py0)), (int(px1), int(py1))], fill=LIGHT_BLUE, width=2)

    # Arrow: Loop Gate -> KLT (dashed)
    for i in range(segments):
        t0 = i / segments
        t1 = (i + 0.5) / segments
        px0 = lg_x+lg_w//2 + (klt_x+klt_w//2 - lg_x-lg_w//2)*t0
        py0 = lg_y+lg_h + (klt_y - lg_y-lg_h)*t0
        px1 = lg_x+lg_w//2 + (klt_x+klt_w//2 - lg_x-lg_w//2)*t1
        py1 = lg_y+lg_h + (klt_y - lg_y-lg_h)*t1
        draw.line([(int(px0), int(py0)), (int(px1), int(py1))], fill=LIGHT_BLUE, width=2)

    # --- Core Insight Callout ---
    ins_x, ins_y, ins_w, ins_h = S2_X0+30, 445, S2_X1-S2_X0-38, 90
    card(draw, ins_x, ins_y, ins_w, ins_h, TEXT_BG3, S2_COLOR, 2, 8)
    draw.text((ins_x+ins_w//2 - draw.textbbox((0,0),"Core Insight",font=FONT_H2)[2]//2,
               ins_y+6), "Core Insight", font=FONT_H2, fill=S2_COLOR)
    draw_multiline(draw, [
        "IMU preintegration rotation  ->  SPA initial value",
        "SPA chi2_final  ->  Adaptive KLT parameter selection",
        "Fail  ->  automatic fallback to default 21x21 KLT",
    ], ins_x+15, ins_y+34, FONT_BODY, WHITE)

    # ===========================
    # SECTION 3: Experiments
    # ===========================
    s3cx = (S3_X0 + S3_X1) // 2
    sx = S3_X0 + 10
    sw = S3_X1 - S3_X0 - 20

    # Big stat: 21.4%
    card(draw, sx, 143, sw, 118, GREEN_DARK, S3_COLOR, 2, 10)
    draw.text((sx+sw//2 - draw.textbbox((0,0),"21.4%",font=FONT_BIG)[2]//2,
               148), "21.4%", font=FONT_BIG, fill=S3_COLOR)
    draw.text((sx+sw//2 - draw.textbbox((0,0),"Frontend Speed Improvement (avg)",font=FONT_SMALL)[2]//2,
               220), "Frontend Speed Improvement (avg)", font=FONT_SMALL, fill=GRAY)
    draw.text((sx+sw//2 - draw.textbbox((0,0),"11 sequences, no regression!",font=FONT_SMALL)[2]//2,
               240), "11 sequences, no regression!", font=FONT_SMALL, fill=(243,156,18))

    # Two side-by-side stats
    hw = sw // 2 - 5
    card(draw, sx, 272, hw, 120, GREEN_DARK, S3_COLOR, 2, 8)
    draw.text((sx+hw//2 - draw.textbbox((0,0),"2.13x",font=FONT_BIG2)[2]//2, 275), "2.13x", font=FONT_BIG2, fill=S3_COLOR)
    draw.text((sx+hw//2 - draw.textbbox((0,0),"SPA Net Gain Ratio",font=FONT_SMALL)[2]//2, 318), "SPA Net Gain Ratio", font=FONT_SMALL, fill=GRAY)
    draw.text((sx+hw//2 - draw.textbbox((0,0),"$1 spent -> $2.13 saved",font=FONT_SMALL)[2]//2, 366), "$1 spent -> $2.13 saved", font=FONT_SMALL, fill=(243,156,18))

    card(draw, sx+hw+10, 272, hw, 120, GREEN_DARK, S3_COLOR, 2, 8)
    draw.text((sx+hw+10+hw//2 - draw.textbbox((0,0),"~3%",font=FONT_BIG2)[2]//2, 275), "~3%", font=FONT_BIG2, fill=S3_COLOR)
    draw.text((sx+hw+10+hw//2 - draw.textbbox((0,0),"KLT Pixel Ops (vs baseline)",font=FONT_SMALL)[2]//2, 318), "KLT Pixel Ops (vs baseline)", font=FONT_SMALL, fill=GRAY)
    draw.text((sx+hw+10+hw//2 - draw.textbbox((0,0),"Lowest tier: 5x5, L=1",font=FONT_SMALL)[2]//2, 366), "Lowest tier: 5x5, L=1", font=FONT_SMALL, fill=(243,156,18))

    # Accuracy card
    card(draw, sx, 405, sw, 118, GREEN_DARK, S3_COLOR, 2, 8)
    draw.text((sx+20, 410), "8/11", font=FONT_BIG2, fill=S3_COLOR)
    draw.text((sx+20, 455), "ATE <= Baseline (no degradation)", font=FONT_SMALL, fill=GRAY)
    draw.text((sx+sw//2+10, 412), "MH_04: RMSE -13.2%", font=FONT_SMALL, fill=(243,156,18))
    draw.text((sx+sw//2+10, 432), "ATE ON/OFF = 0.95x (avg)", font=FONT_SMALL, fill=GRAY)
    draw.text((sx+sw//2+10, 452), "V201_easy: +45.8% speedup", font=FONT_SMALL, fill=GRAY)
    draw.text((sx+sw//2+10, 472), "MH_01_easy: +29.1% speedup", font=FONT_SMALL, fill=GRAY)

    # Limitation
    card(draw, sx, 535, sw, 90, RED_DARK, (231,76,60), 2, 8)
    draw.text((sx+sw//2 - draw.textbbox((0,0),"Limitation (V203 difficult)",font=FONT_SMALL)[2]//2,
               540), "Limitation (V203 difficult)", font=FONT_SMALL, fill=(231,76,60))
    draw_multiline(draw, [
        "V203 ON/OFF = 1.70x  (better than original 2.33x)",
        "Failure: large displacement + IMU drift + low texture.",
        "Architecture-level change needed (not just tuning)."
    ], sx+8, 565, FONT_SMALL, GRAY)

    # Dataset badge
    chip(draw, sx, 635, sw, 40, "Dataset: EuRoC MAV  |  MH_01-05, V101-103, V201-203  |  11 seqs",
         FONT_SMALL, LIGHT_BLUE, BLUE_DARK, 6)

    # Integration badge
    chip(draw, sx, 685, sw, 50,
         "Integration: Zero-invasive enhancement on VINS-Mono | No changes to BA, IMU, pose graph",
         FONT_SMALL, LIGHT_BLUE, BLUE_DARK, 6)

    # ===========================
    # SECTION 4: Applications
    # ===========================
    sx4 = S4_X0 + 10
    sw4 = S4_X1 - S4_X0 - 20

    # Conditions card
    card(draw, sx4, 143, sw4, 195, ORANGE_DARK, S4_COLOR, 2, 8)
    draw.text((sx4+sw4//2 - draw.textbbox((0,0),"SPA-KLT Valid When:",font=FONT_H2)[2]//2,
               148), "SPA-KLT Valid When:", font=FONT_H2, fill=S4_COLOR)
    conds = [
        "  Moderate inter-frame motion (not too small, not too large)",
        "  Rich, high-contrast texture (reliable gradient signal)",
        "  Lens distortion calibrated or controlled",
        "  Stable exposure (or affine absorption works)",
        "  Strong-response corner features present",
    ]
    for i, c in enumerate(conds):
        draw.text((sx4+10, 178+i*24), c, font=FONT_BODY, fill=WHITE)
    draw.text((sx4+sw4//2 - draw.textbbox((0,0),"All 5 conditions = sufficient (8/11 EuRoC)",font=FONT_SMALL)[2]//2,
               298), "All 5 conditions = sufficient (8/11 EuRoC)", font=FONT_SMALL, fill=GRAY)

    # Application domains
    card(draw, sx4, 350, sw4, 265, BLUE_MID, (52, 152, 219), 2, 10)
    draw.text((sx4+sw4//2 - draw.textbbox((0,0),"Application Scenarios",font=FONT_H2)[2]//2,
               355), "Application Scenarios", font=FONT_H2, fill=(52, 152, 219))

    # 4 app icons in a row
    apps = [("MAVs", "Micro Aerial\nVehicles"), ("AR", "Augmented\nReality"),
            ("Robots", "Autonomous\nRobots"), ("Nav", "Indoor\nNavigation")]
    icon_w = sw4 // 4
    for i, (label, desc) in enumerate(apps):
        ix = sx4 + i*icon_w + icon_w//2
        # circle icon placeholder
        draw.ellipse([ix-22, 392, ix+22, 436], fill=S4_COLOR)
        draw.text((ix - draw.textbbox((0,0),label,font=FONT_SMALL)[2]//2, 396), label, font=FONT_SMALL, fill=WHITE)
        draw.text((ix - draw.textbbox((0,0),desc,font=FONT_SMALL)[2]//2, 442), desc, font=FONT_SMALL, fill=GRAY)

    # Value bullets
    values = [
        "  Zero-invasive: plug into any optical-flow VIO",
        "  Embedded-friendly: KLT no longer the bottleneck",
        "  Quantifiable: chi2 threshold makes it observable",
        "  Fallback-guaranteed: fail-safe to default KLT",
    ]
    for i, v in enumerate(values):
        draw.text((sx4+10, 490+i*22), v, font=FONT_BODY, fill=(169,204,227))

    # Future directions
    card(draw, sx4, 628, sw4, 165, BLUE_MID, (93,173,226), 2, 8)
    draw.text((sx4+sw4//2 - draw.textbbox((0,0),"Future Directions",font=FONT_H2)[2]//2,
               632), "Future Directions", font=FONT_H2, fill=(93,173,226))
    futures = [
        "-> Learning-based feature points to cross SPA boundary",
        "-> Multi-camera / event-camera extension",
        "-> LET-NET: illumination-invariant features for KLT",
        "-> DSInit: extend SPA to VIO initialization phase",
        "-> Co-deploy with LEVIO hardware parallelization",
    ]
    for i, f in enumerate(futures):
        draw.text((sx4+10, 658+i*24), f, font=FONT_BODY, fill=WHITE)

    # ---- Vertical section labels ----
    # Draw on the side of each divider
    section_labels = [
        ("① Pain Points", S1_X1-7+4, (S1_X0+S1_X1)//2, S1_COLOR),
        ("② SPA-KLT Framework", S2_X1-7+4, (S2_X0+S2_X1)//2, S2_COLOR),
        ("③ Experiments", S3_X1-7+4, (S3_X0+S3_X1)//2, S3_COLOR),
        ("④ Applications", S4_X1-7+4, (S4_X0+S4_X1)//2, S4_COLOR),
    ]
    for text, x, cx, color in section_labels:
        pass  # section labels are implicit via the colored divider bars

    out_path = os.path.join(os.path.dirname(__file__), "spa-klt-graphical-abstract.png")
    img.save(out_path, "PNG", optimize=False)
    print(f"Saved: {out_path}  ({img.size[0]}x{img.size[1]})")
    return out_path


if __name__ == "__main__":
    render()
