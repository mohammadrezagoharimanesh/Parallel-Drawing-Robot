import tkinter as tk
from tkinter import filedialog, messagebox
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import serial, time, ezdxf, cv2
from PIL import Image, ImageDraw, ImageFont
from arabic_reshaper import reshape
from bidi.algorithm import get_display


d = 48.0
L1 = 108.8
L2 = 131.2
xL, yL = -d/2, 0.0
xR, yR = d/2, 0.0

BOX_W, BOX_H = 250.0, 100.0
BOX_CX, BOX_CY = 0.0, 118.0
STEP = 1.0
SEND_DELAY = 0.015
SERIAL_DECIMALS = 0
REFRESH_EVERY = 4

try:
    arduino = serial.Serial('COM5', 9600, timeout=1)
    time.sleep(2)
    print("✅ Arduino Connected")
except Exception:
    arduino = None
    print("⚠️ Simulation mode")

path_x, path_y = [], []


def ik(x, y):
    dL = np.hypot(x - xL, y - yL)
    if not (abs(L1-L2) <= dL <= L1+L2): return None, None
    cosB = (L1**2 + dL**2 - L2**2) / (2*L1*dL)
    t1 = np.arctan2(y - yL, x - xL) + np.arccos(np.clip(cosB, -1, 1))

    dR = np.hypot(x - xR, y - yR)
    if not (abs(L1-L2) <= dR <= L1+L2): return None, None
    cosB = (L1**2 + dR**2 - L2**2) / (2*L1*dR)
    t2 = np.arctan2(y - yR, x - xR) - np.arccos(np.clip(cosB, -1, 1))
    return np.degrees(t1), np.degrees(t2)


def init_plot():
    global line_path, arm_l, arm_r
    x = np.linspace(-200, 200, 401)
    y = np.linspace(-50, 300, 351)
    X, Y = np.meshgrid(x, y)
    dL = np.hypot(X - xL, Y - yL)
    dR = np.hypot(X - xR, Y - yR)
    lo, hi = abs(L1-L2), L1+L2
    reach = (dL >= lo) & (dL <= hi) & (dR >= lo) & (dR <= hi)
    ax.contour(X, Y, reach, levels=[0.5], colors='lightgray', linewidths=1.5, linestyles='dashed')
    ax.set(xlim=(-200, 200), ylim=(-50, 300))
    ax.set_aspect('equal')
    ax.grid(True)
    line_path, = ax.plot([], [], 'purple', lw=2)
    arm_l, = ax.plot([], [], 'bo-', lw=3)
    arm_r, = ax.plot([], [], 'go-', lw=3)


def move(x, y, draw=True, refresh=True):
    deg1, deg2 = ik(x, y)
    if deg1 is None: return False
    if draw:
        path_x.append(x); path_y.append(y)

    if refresh:
        r1, r2 = np.radians(deg1), np.radians(deg2)
        mxL = xL + L1*np.cos(r1); myL = yL + L1*np.sin(r1)
        mxR = xR + L1*np.cos(r2); myR = yR + L1*np.sin(r2)
        arm_l.set_data([xL, mxL, x], [yL, myL, y])
        arm_r.set_data([xR, mxR, x], [yR, myR, y])
        line_path.set_data(path_x, path_y)
        canvas.draw_idle(); root.update()

    if arduino:
        f = SERIAL_DECIMALS
        arduino.write(f"{deg1:.{f}f},{deg2:.{f}f}\n".encode())
        time.sleep(SEND_DELAY)
    return True


def resample(p, step):
    seg = np.hypot(*np.diff(p, axis=0).T)
    s = np.concatenate([[0], np.cumsum(seg)])
    if s[-1] < 1e-9: return p[:1]
    n = max(int(s[-1] / step), 1)
    t = np.linspace(0, s[-1], n + 1)
    return np.column_stack([np.interp(t, s, p[:, 0]), np.interp(t, s, p[:, 1])])


def smooth_closed(p, sigma):
    k = np.arange(-int(3*sigma), int(3*sigma) + 1)
    pad = len(k) // 2
    if len(p) < 2*pad + 2: return p
    g = np.exp(-k**2 / (2*sigma**2)); g /= g.sum()
    q = np.vstack([p[-pad:], p, p[:pad]])
    return np.column_stack([np.convolve(q[:, i], g, mode='valid') for i in range(2)])


def order_closed(polys, start):
    out, cur, rest = [], np.array(start, float), list(polys)
    while rest:
        j = min(range(len(rest)), key=lambda i: np.min(np.hypot(*(rest[i][:-1] - cur).T)))
        q = rest.pop(j)[:-1]
        k = int(np.argmin(np.hypot(*(q - cur).T)))
        q = np.roll(q, -k, axis=0)
        p = np.vstack([q, q[:1]])
        out.append(p)
        cur = p[-1]
    return out


def trace(polys):
    global path_x, path_y
    path_x, path_y = [], []
    seq, cur = [], None
    for p in polys:
        p = resample(p, STEP)
        if cur is not None:
            travel = resample(np.array([cur, p[0]]), STEP)
            seq += [(x, y, False) for x, y in travel[1:]]
        seq += [(x, y, True) for x, y in p]
        cur = p[-1]
    bad, prev = 0, False
    for i, (x, y, pen) in enumerate(seq):
        if pen and not prev:
            path_x.append(np.nan); path_y.append(np.nan)
        prev = pen
        if not move(x, y, pen, refresh=(i % REFRESH_EVERY == 0 or i == len(seq) - 1)):
            bad += 1
    return bad


def text_polylines(txt):
    font = None
    for name in ("tahoma.ttf", "arial.ttf", "DejaVuSans.ttf"):
        try:
            font = ImageFont.truetype(name, 220); break
        except OSError:
            continue
    if font is None:
        font = ImageFont.load_default(220)
    s = get_display(reshape(txt))
    l, t, r, b = font.getbbox(s)
    pad = 20
    img = Image.new('L', (int(r - l) + 2*pad, int(b - t) + 2*pad), 0)
    ImageDraw.Draw(img).text((pad - l, pad - t), s, 255, font=font)

    cs, _ = cv2.findContours(np.array(img), cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    polys = []
    for c in cs:
        if cv2.contourArea(c) < 30 or len(c) < 12: continue
        p = smooth_closed(c[:, 0, :].astype(float), 3.0)
        polys.append(np.vstack([p, p[:1]]))
    if not polys: return []

    pts = np.vstack(polys)
    mn, mx = pts.min(axis=0), pts.max(axis=0)
    scale = min(BOX_W / max(mx[0] - mn[0], 1e-9), BOX_H / max(mx[1] - mn[1], 1e-9))
    c0 = (mn + mx) / 2
    polys = [np.column_stack([(p[:, 0] - c0[0]) * scale + BOX_CX,
                              BOX_CY - (p[:, 1] - c0[1]) * scale]) for p in polys]
    return order_closed(polys, (BOX_CX, BOX_CY))


def text_to_path():
    txt = text_entry.get()
    if not txt:
        messagebox.showwarning("خطا", "لطفاً متن را وارد کنید")
        return
    polys = text_polylines(txt)
    if not polys:
        messagebox.showwarning("خطا", "متن قابل تشخیص نیست")
        return
    bad = trace(polys)
    if bad:
        messagebox.showwarning("هشدار", f"{bad} نقطه خارج از فضای کاری بود")


def load_dxf():
    f = filedialog.askopenfilename(filetypes=[("DXF", "*.dxf")])
    if not f: return
    try:
        doc = ezdxf.readfile(f)
        polys = [np.array([[e.dxf.start.x, e.dxf.start.y + 120],
                           [e.dxf.end.x, e.dxf.end.y + 120]])
                 for e in doc.modelspace().query('LINE')]
        trace(polys)
        messagebox.showinfo("موفقیت", "DXF ترسیم شد")
    except Exception as e:
        messagebox.showerror("خطا", str(e))


if __name__ == "__main__":
    root = tk.Tk()
    root.title("گوهری منش")
    root.geometry("850x550")

    ctrl = tk.Frame(root, width=320, padx=10, pady=10)
    ctrl.pack(side=tk.LEFT, fill=tk.Y)

    tk.Label(ctrl, text="متن:").pack(anchor=tk.W)
    text_entry = tk.Entry(ctrl, width=35, font=("Tahoma", 11))
    text_entry.pack(pady=5)

    tk.Button(ctrl, text="ترسیم متن", command=text_to_path, bg="#4CAF50", fg="white", width=25).pack(pady=5)
    tk.Button(ctrl, text="بارگذاری DXF", command=load_dxf, bg="#2196F3", fg="white", width=25).pack(pady=5)
    tk.Label(ctrl, text="").pack()
    tk.Label(ctrl, text="طراح: محمدرضا گوهری‌منش", font=("Tahoma", 10, "bold")).pack(anchor=tk.W)
    tk.Label(ctrl, text="استاد: دکتر مسعود گوهری‌منش", font=("Tahoma", 10, "bold")).pack(anchor=tk.W)
    tk.Label(ctrl, text="دانشگاه: تربت حیدریه", font=("Tahoma", 10, "bold")).pack(anchor=tk.W)

    plot_frame = tk.Frame(root)
    plot_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

    fig, ax = plt.subplots(figsize=(5, 5))
    canvas = FigureCanvasTkAgg(fig, master=plot_frame)
    canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    init_plot()
    move(0, 150, draw=False)
    root.mainloop()
