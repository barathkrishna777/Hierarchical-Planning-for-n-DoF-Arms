import numpy as np
import subprocess
import pandas as pd
import os
from timeit import default_timer as timer

# Configuration
test_maps = ["maps/map7_blocks_grid.txt",
    "maps/map8_long_vertical_blocks.txt"]
planners = [0, 1, 2]  # planner IDs
dofs = 6
tests_per_map = 4
repeats = 5
tmp_solution = "tmp.txt"

# Log file for vertices counts
vertices_log_file = "vertices_log.txt"
# Initialize/clear the vertices log
open(vertices_log_file, 'w').close()


def convertPIs(aString):
    if aString.endswith(","):
        aString = aString[:-1]
    aString = aString.replace("pi", "3.141592")
    return [str(round(eval(expr), 2)) for expr in aString.split(",")]


def is_valid_configuration(config, num_dofs, map_file):
    cmd = f"./config_checker.out {map_file} {num_dofs} {config}"
    return subprocess.run(cmd.split(), check=False).returncode == 0


def generate_random_configuration(num_dofs, map_file, max_trials=10000):
    for _ in range(max_trials):
        cfg = np.round(np.random.uniform(0, 2*np.pi, num_dofs), 2)
        cfg_str = ",".join(map(str, cfg))
        if is_valid_configuration(cfg_str, num_dofs, map_file):
            return cfg_str
    raise RuntimeError(f"No valid configuration after {max_trials} trials")


def parse_solution(path):
    with open(path) as f:
        lines = [l.strip() for l in f if l.strip()]
    sol_rows = []
    vertices = None
    for line in lines:
        if line.lower().startswith("vertices:"):
            try:
                vertices = int(line.split(':',1)[1].strip())
            except:
                vertices = None
        elif ',' in line:
            sol_rows.append(line.rstrip(","))
    if len(sol_rows) < 2:
        return None, vertices
    data = np.array([list(map(float, r.split(","))) for r in sol_rows])
    difs = np.abs(data[1:] - data[:-1])
    cost = np.minimum(difs, 2*np.pi - difs).sum()
    return round(float(cost), 2), vertices


def run_tests():
    for map_file in test_maps:
        base = os.path.splitext(os.path.basename(map_file))[0]
        records = []
        # Generate test cases
        cases = [(generate_random_configuration(dofs, map_file),
                  generate_random_configuration(dofs, map_file))
                 for _ in range(tests_per_map)]

        for planner_id in planners:
            for idx, (start_cfg, goal_cfg) in enumerate(cases):
                start_pi = ",".join(convertPIs(start_cfg))
                goal_pi = ",".join(convertPIs(goal_cfg))
                times, costs, verts = [], [], []
                successes = 0

                for rep in range(repeats):
                    # Run planner
                    plan_cmd = f"./planner.out {map_file} {dofs} {start_pi} {goal_pi} {planner_id} {tmp_solution}"
                    t0 = timer()
                    ok = subprocess.run(plan_cmd.split(), check=False).returncode == 0
                    times.append(round(timer() - t0, 2))

                    cost, vert = None, None
                    if ok:
                        # Verify
                        verify_cmd = f"./verifier.out {map_file} {dofs} {start_pi} {goal_pi} {tmp_solution}"
                        if subprocess.run(verify_cmd.split(), check=False).returncode == 0:
                            successes += 1
                            cost, vert = parse_solution(tmp_solution)
                    costs.append(cost)
                    verts.append(vert)

                    # Log this run's vertices
                    with open(vertices_log_file, 'a') as vf:
                        vf.write(f"{map_file},{planner_id},{idx},{rep},{vert}\n")

                # Aggregate stats
                t_arr = np.array([t for t in times if t is not None])
                c_arr = np.array([c for c in costs if c is not None])
                v_arr = np.array([v for v in verts if v is not None])
                rec = {
                    'planner': planner_id,
                    'map': base,
                    'test_idx': idx,
                    'mean_time': round(t_arr.mean(), 2) if len(t_arr) else None,
                    'std_time': round(t_arr.std(), 2) if len(t_arr) else None,
                    'mean_cost': round(c_arr.mean(), 2) if len(c_arr) else None,
                    'std_cost': round(c_arr.std(), 2) if len(c_arr) else None,
                    'mean_vertices': round(v_arr.mean(), 2) if len(v_arr) else None,
                    'std_vertices': round(v_arr.std(), 2) if len(v_arr) else None,
                    'success_rate': round(successes/repeats, 2),
                    'start_cfg': start_cfg,
                    'goal_cfg': goal_cfg
                }
                records.append(rec)

        # Save per-map CSV
        df = pd.DataFrame(records)
        out_name = f"{base}_results.csv"
        df.to_csv(out_name, index=False)
        print(f"Saved results to {out_name}")

if __name__ == '__main__':
    run_tests()
