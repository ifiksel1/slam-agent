#!/usr/bin/env python3
"""XTest input for display :0 (no xdotool).
usage: xin.py click X Y | move X Y | key COMBO [COMBO...] | type TEXT | where
COMBO like ctrl+w, space, Return, bracketleft, ctrl+shift+p (X keysym names)."""
import ctypes, sys, time
x11 = ctypes.CDLL("libX11.so.6"); xt = ctypes.CDLL("libXtst.so.6")
x11.XOpenDisplay.restype = ctypes.c_void_p
x11.XStringToKeysym.restype = ctypes.c_ulong
x11.XDefaultRootWindow.restype = ctypes.c_ulong
d = ctypes.c_void_p(x11.XOpenDisplay(b":0"))
MOD = {"ctrl": "Control_L", "shift": "Shift_L", "alt": "Alt_L", "super": "Super_L"}
def kc(name):
    ks = x11.XStringToKeysym(name.encode())
    if not ks: raise SystemExit("bad keysym " + name)
    return x11.XKeysymToKeycode(d, ctypes.c_ulong(ks))
def ev(code, down):
    xt.XTestFakeKeyEvent(d, code, down, 0); x11.XFlush(d); time.sleep(0.03)
def combo(c):
    parts = c.split("+"); mods = [kc(MOD.get(p, p)) for p in parts[:-1]]; k = kc(parts[-1])
    for m in mods: ev(m, 1)
    ev(k, 1); ev(k, 0)
    for m in reversed(mods): ev(m, 0)
    time.sleep(0.1)
SPECIAL = {" ": "space", "/": "slash", ".": "period", "_": "shift+underscore", "-": "minus", ":": "shift+colon", "~": "shift+asciitilde"}
cmd = sys.argv[1]
if cmd in ("click", "move"):
    x, y = int(sys.argv[2]), int(sys.argv[3])
    # Electron needs a few motion events and a long-ish press to register a click
    for dx in (-10, -5, 0):
        xt.XTestFakeMotionEvent(d, -1, x + dx, y, 0); x11.XSync(d, 0); time.sleep(0.1)
    if cmd == "click":
        xt.XTestFakeButtonEvent(d, 1, 1, 0); x11.XSync(d, 0); time.sleep(0.2)
        xt.XTestFakeButtonEvent(d, 1, 0, 0); x11.XSync(d, 0)
elif cmd == "key":
    for c in sys.argv[2:]: combo(c)
elif cmd == "type":
    for ch in sys.argv[2]:
        if ch in SPECIAL: combo(SPECIAL[ch])
        elif ch.isupper(): combo("shift+" + ch)
        else: combo(ch)
