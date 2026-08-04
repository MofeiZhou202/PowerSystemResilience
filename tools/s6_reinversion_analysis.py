import sys, re, math

rows = []
for line in sys.stdin:
    if "DS-REINVERT-MODEL" not in line:
        continue
    d = dict(re.findall(r"(\w+)=([-0-9.e+]+)", line))
    ph = "II" if "phase=II" in line else "I"
    R = float(d.get("R_sec", 0))
    u = float(d.get("u_sec", 0))
    it = int(float(d.get("iterations", 0)))
    K = float(d.get("interval", 0))
    if R <= 0 or u <= 0 or K <= 0 or it < 200:
        continue
    Kstar = math.sqrt(2 * R / u)
    cost = lambda k: R / k + u * k / 2.0
    ratio = cost(K) / cost(Kstar)
    rows.append((ph, it, K, Kstar, ratio))

rows.sort(key=lambda r: -r[1])
print("%3s %7s %6s %8s %10s" % ("ph", "iters", "cur_K", "K*", "cost_ratio"))
for ph, it, K, Ks, ra in rows:
    print("%3s %7d %6.0f %8.1f %10.3f" % (ph, it, K, Ks, ra))
if rows:
    mx = max(r[4] for r in rows)
    print("--- max cost(cur)/cost(K*) = %.3fx  (2x promotion gate: %s)"
          % (mx, "PASS" if mx >= 2.0 else "FAIL"))
    print("--- cases measured: %d ; phase-II median K* = %.0f"
          % (len(rows),
             sorted(r[3] for r in rows if r[0] == "II")[len([r for r in rows if r[0]=="II"])//2]))
