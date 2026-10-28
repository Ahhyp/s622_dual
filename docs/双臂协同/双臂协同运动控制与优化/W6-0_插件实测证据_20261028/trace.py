#!/usr/bin/env python3
"""High-rate trace of DetachableJoint attach/detach vs child pose (Fortress 6.16)."""
import json, os, re, subprocess, sys, threading, time
ENV = dict(os.environ); ENV["HOME"] = "/tmp/w6probe/home"
WF = sys.argv[1]; LABEL = sys.argv[2]
OUT = {"world": os.path.basename(WF)}
def run(c, t=20):
    p = subprocess.run(c, shell=True, capture_output=True, text=True, timeout=t, env=ENV); return p.returncode, p.stdout
def z():
    rc, out = run("ign model -m probe_child -p")
    m = re.findall(r"^\s{4}\[([-\d.eE+ ]+)\]", out, re.M)
    return round(float(m[0].split()[2]), 6) if m else None

log = open(f"/tmp/w6probe/trace_{LABEL}.log", "w")
srv = subprocess.Popen(["ign","gazebo","-s","-r","-v","4",WF], stdout=log, stderr=subprocess.STDOUT, env=ENV)
t0 = time.time()
while time.time() - t0 < 25:
    rc, out = run("ign model --list")
    if rc == 0 and "probe_child" in out: break
    time.sleep(0.2)
OUT["server_up_s"] = round(time.time() - t0, 3)

events = []
stop = threading.Event()
def sub_state():
    p = subprocess.Popen(["ign","topic","-e","-t","/probe/state"], stdout=subprocess.PIPE,
                         stderr=subprocess.STDOUT, text=True, env=ENV)
    for line in p.stdout:
        s = line.strip()
        if s.startswith("data:"):
            events.append((round(time.time() - t0, 3), s.split(":",1)[1].strip().strip('"')))
        if stop.is_set(): break
    p.terminate()

th = threading.Thread(target=sub_state, daemon=True); th.start()

samples = []
def sample_until(until_t):
    while time.time() - t0 < until_t:
        samples.append((round(time.time() - t0, 3), z()))
        time.sleep(0.02)

def pub(topic):
    rc, out = run(f"ign topic -t {topic} -m ignition.msgs.Empty -p ''", 15)
    return {"rc": rc, "at_s": round(time.time() - t0, 3)}

actions = []
sample_until(3.0)
actions.append({"detach": pub("/probe/detach")})
sample_until(5.0)
actions.append({"attach": pub("/probe/attach")})
sample_until(7.0)
actions.append({"detach2": pub("/probe/detach")})
sample_until(9.0)
stop.set(); time.sleep(0.3)
srv.terminate()
try: srv.wait(timeout=8)
except: srv.kill()
OUT["actions"] = actions
OUT["state_events"] = events
OUT["samples"] = samples
json.dump(OUT, open(f"/tmp/w6probe/trace_{LABEL}.json","w"))
# summary
zs = [z for _, z in samples if z is not None]
print(json.dumps({"label": LABEL, "server_up_s": OUT["server_up_s"], "n_samples": len(zs),
                  "z_first": zs[0] if zs else None, "z_last": zs[-1] if zs else None,
                  "events": events, "actions": actions}, indent=1))
