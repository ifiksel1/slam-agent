#!/usr/bin/env python3
"""X window helper for display :0 (EWMH, no wmctrl/xdotool).
usage: xwin.py list | active | fg_active | activate_fg
  list         every managed window: id, WM_CLASS, title
  active       the active window: id, WM_CLASS, title
  fg_active    exit 0 if the active window is Foxglove (prints its title), else 1
  activate_fg  activate (raise + focus) the Foxglove window; exit 0 if it then is active
  activate_title SUBSTR  activate the newest window whose title contains SUBSTR"""
import ctypes
import sys
import time

x11 = ctypes.CDLL("libX11.so.6")
x11.XOpenDisplay.restype = ctypes.c_void_p
x11.XDefaultRootWindow.restype = ctypes.c_ulong
x11.XDefaultRootWindow.argtypes = [ctypes.c_void_p]
x11.XInternAtom.restype = ctypes.c_ulong
x11.XInternAtom.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int]
x11.XGetWindowProperty.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.c_ulong, ctypes.c_long, ctypes.c_long,
                                   ctypes.c_int, ctypes.c_ulong, ctypes.POINTER(ctypes.c_ulong),
                                   ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_ulong),
                                   ctypes.POINTER(ctypes.c_ulong), ctypes.POINTER(ctypes.c_void_p)]
x11.XFree.argtypes = [ctypes.c_void_p]
d = ctypes.c_void_p(x11.XOpenDisplay(b":0"))
if not d.value:
    sys.exit("cannot open :0")
root = x11.XDefaultRootWindow(d)
A = lambda n: x11.XInternAtom(d, n.encode(), 0)  # noqa: E731


def prop(w, name, typ=0):
    t, f, n, after, data = ctypes.c_ulong(), ctypes.c_int(), ctypes.c_ulong(), ctypes.c_ulong(), ctypes.c_void_p()
    if x11.XGetWindowProperty(d, w, A(name), 0, 1 << 20, 0, typ, ctypes.byref(t), ctypes.byref(f), ctypes.byref(n),
                              ctypes.byref(after), ctypes.byref(data)) != 0 or not data.value:
        return None, 0, 0
    if f.value == 8:
        out = ctypes.string_at(data.value, n.value)
    elif f.value == 32:
        out = list((ctypes.c_ulong * n.value).from_address(data.value))
    else:
        out = None
    x11.XFree(data)
    return out, f.value, n.value


def info(w):
    cls = (prop(w, "WM_CLASS")[0] or b"").replace(b"\0", b" ").decode(errors="replace").strip()
    title = (prop(w, "_NET_WM_NAME")[0] or prop(w, "WM_NAME")[0] or b"").decode(errors="replace")
    return cls, title


def clients():
    return prop(root, "_NET_CLIENT_LIST")[0] or []


def active():
    v = prop(root, "_NET_ACTIVE_WINDOW")[0]
    return v[0] if v else 0


def is_fg(w):
    return "foxglove" in info(w)[0].lower()


def fg_window():
    ws = [w for w in clients() if is_fg(w)]
    return ws[-1] if ws else 0


class XClientMessageEvent(ctypes.Structure):
    _fields_ = [("type", ctypes.c_int), ("serial", ctypes.c_ulong), ("send_event", ctypes.c_int),
                ("display", ctypes.c_void_p), ("window", ctypes.c_ulong), ("message_type", ctypes.c_ulong),
                ("format", ctypes.c_int), ("data", ctypes.c_long * 5)]


class XEvent(ctypes.Union):
    _fields_ = [("xclient", XClientMessageEvent), ("pad", ctypes.c_long * 24)]


def activate(w):
    ev = XEvent()
    ev.xclient.type = 33  # ClientMessage
    ev.xclient.send_event = 1
    ev.xclient.window = w
    ev.xclient.message_type = A("_NET_ACTIVE_WINDOW")
    ev.xclient.format = 32
    ev.xclient.data[0] = 2  # source: pager, so the WM honours it
    ev.xclient.data[1] = 0
    x11.XSendEvent.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.c_int, ctypes.c_long, ctypes.c_void_p]
    x11.XSendEvent(d, root, 0, (1 << 20) | (1 << 19), ctypes.byref(ev))
    x11.XFlush(d)


cmd = sys.argv[1] if len(sys.argv) > 1 else "active"
if cmd == "list":
    for w in clients():
        print(hex(w), *info(w), sep="\t")
elif cmd == "active":
    w = active()
    print(hex(w), *(info(w) if w else ("", "")), sep="\t")
elif cmd == "fg_active":
    w = active()
    ok = bool(w) and is_fg(w)
    print(info(w)[1] if ok else "not foxglove: " + "\t".join(info(w) if w else ("none",)))
    sys.exit(0 if ok else 1)
elif cmd == "activate_fg":
    w = fg_window()
    if not w:
        sys.exit("no foxglove window")
    for _ in range(3):
        activate(w)
        time.sleep(0.5)
        if active() == w:
            print(info(w)[1])
            sys.exit(0)
    sys.exit("could not activate foxglove (active: %s)" % "\t".join(info(active())))
elif cmd == "activate_title":   # activate_title SUBSTRING: activate the newest window whose title contains it
    ws = [w for w in clients() if sys.argv[2] in info(w)[1]]
    if not ws:
        sys.exit("no window titled *%s*" % sys.argv[2])
    activate(ws[-1])
    time.sleep(0.5)
    print("\t".join(info(active())))
    sys.exit(0 if active() == ws[-1] else 1)
