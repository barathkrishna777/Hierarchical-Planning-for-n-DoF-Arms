import os
import numpy as np
import subprocess
import pandas as pd
from timeit import default_timer as timer

def convertPIs(aString):
    if aString[-1] == ",":
        aString = aString[:-1]
    aString = aString.replace("pi", "3.141592")
    return [str(round(eval(expr), 2)) for expr in aString.split(",")]

def is_valid_configuration(cfg, dofs, map_file):
    cmd = f"./config_checker.out {map_file} {dofs} {cfg}"
    return subprocess.run(cmd.split(), check=False).returncode == 0

def generate_random_configuration(dofs, map_file, max_trials=10000):
    for _ in range(max_trials):
        cfg = np.round(np.random.uniform(0, 2*np.pi, dofs), 2)
        cfg_str = ",".join(map(str, cfg))
        if is_valid_configuration(cfg_str, dofs, map_file):
            return cfg_str
    raise RuntimeError(f"No valid config after {max_trials} trials on {map_file}")

def run_test(exe, map_file, dofs, start_str, goal_str, pid, tmp="tmp.txt"):
    plan = f"{exe} {map_file} {dofs} {start_str} {goal_str} {pid} {tmp}"
    verify = f"./verifier.out {map_file} {dofs} {start_str} {goal_str} {tmp}"
    try:
        t0 = timer()
        subprocess.run(plan.split(), check=True)
        t = round(timer() - t0, 2)
        ok = subprocess.run(verify.split(), check=False).returncode == 0
        if not ok:
            return False, t, -1.0, -1
        sol = np.loadtxt(tmp, delimiter=",", skiprows=1)[:, :-1]
        diffs = np.abs(sol[1:] - sol[:-1])
        cost = round(np.minimum(diffs, 2*np.pi-diffs).sum(), 2)
        verts = int(sol.shape[0])
        return True, t, cost, verts
    except:
        return False, 0.0, -1.0, -1

def grade_map(exe, map_file, planners, dofs=6, cases=4, repeats=5, out_dir="results"):
    os.makedirs(out_dir, exist_ok=True)
    base = os.path.splitext(os.path.basename(map_file))[0]
    rows = []
    # generate cases
    tests = [(generate_random_configuration(dofs, map_file),
              generate_random_configuration(dofs, map_file))
             for _ in range(cases)]
    for pid in planners:
        for idx,(s,g) in enumerate(tests):
            s_str = ",".join(convertPIs(s))
            g_str = ",".join(convertPIs(g))
            ts, cs, vs, succ = [], [], [], 0
            for _ in range(repeats):
                ok,t,c,v = run_test(exe, map_file, dofs, s_str, g_str, pid)
                ts.append(t); cs.append(c); vs.append(v); succ+=ok
            rows.append({
                "planner":     pid,
                "mapName":     base,
                "caseIndex":   idx,
                "numDOFs":     dofs,
                "startConfig": s_str,
                "goalConfig":  g_str,
                "meanTime":    round(np.mean(ts),2),
                "stdTime":     round(np.std(ts),2),
                "meanCost":    round(np.mean(cs),2),
                "stdCost":     round(np.std(cs),2),
                "meanVerts":   round(np.mean(vs),2),
                "stdVerts":    round(np.std(vs),2),
                "successRate": round(succ/repeats,2)
            })
    df = pd.DataFrame(rows)
    out_csv = os.path.join(out_dir, f"{base}_results.csv")
    df.to_csv(out_csv, index=False)
    print(f"Wrote {out_csv}")

if __name__ == "__main__":
    EXEC = "./planner.out"
    MAPS = [
        "map1_fine.txt",
        "map2_fine.txt",
        "map3_horizontal.txt",
        "map4_vertical.txt",
        "map5_diagonal.txt",
        "map6_zigzag.txt",
        "map7_blocks_grid.txt",
        "map8_long_vertical_blocks.txt"
    ]
    PLANNERS = [0, 1, 2]  # your planner IDs

    for m in MAPS:
        grade_map(EXEC, m, PLANNERS)
