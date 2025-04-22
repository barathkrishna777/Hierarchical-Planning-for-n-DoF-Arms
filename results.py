import numpy as np
import argparse
import subprocess
import pandas as pd
from timeit import default_timer as timer
import random
import os

def convertPIs(aString):
    if aString[-1] == ",":
        aString = aString[:-1]
    aString = aString.replace("pi", "3.141592")
    vecOfStrings = aString.split(",")
    return [str(round(eval(anExpression), 2)) for anExpression in vecOfStrings]  # Round to 2 decimal places

def generate_random_configuration(num_dofs, map_file):
    attempts = 0
    while True:
        config = np.round(np.random.uniform(0, 2 * np.pi, num_dofs), 2)
        config_str = ",".join(map(str, config))
        attempts += 1
        print(f"Attempt {attempts}: Trying config {config_str}")
        if is_valid_configuration(config_str, num_dofs, map_file):
            print(f"Valid config found after {attempts} attempts")
            return config_str

def is_valid_configuration(config, num_dofs, map_file):
    command = "./config_checker.out {} {} {}".format(map_file, num_dofs, config)
    result = subprocess.run(command.split(" "), check=False).returncode
    return result == 0

def graderMain(executablePath, gradingCSV):
    random.seed(42)
    np.random.seed(42)
    # maps = ["maps/map1_fine.txt", "maps/map2_fine.txt", 
    #         "maps/map3_fine.txt", "maps/map4_fine.txt"]
    maps = ["robot_maps/map_challenging.txt"]
    num_problems = 5
    num_repeats = 4
    planners = [0, 1, 2]
    scores = []

    test_cases = []
    for _ in range(num_problems):
        map_file = random.choice(maps)
        num_dofs = 6
        start = generate_random_configuration(num_dofs, map_file)
        goal = generate_random_configuration(num_dofs, map_file)
        test_cases.append((map_file, num_dofs, start, goal))

        print(f"Generated test case: {map_file}, DOFs: {num_dofs}, Start: {start}, Goal: {goal}")

    for aPlanner in planners:
        for i, (map_file, num_dofs, start, goal) in enumerate(test_cases):
            outputSolutionFile = f"tmp_{aPlanner}_{i}.txt.txt"
            startPosString = ",".join(convertPIs(start))
            goalPosString = ",".join(convertPIs(goal))
            times, costs, vertices, successes = [], [], [], 0

            for _ in range(num_repeats):
                commandPlan = "{} {} {} {} {} {} {}".format(
                    executablePath, map_file, num_dofs, startPosString, goalPosString, aPlanner, outputSolutionFile)
                commandVerify = "./verifier.out {} {} {} {} {}".format(
                    map_file, num_dofs, startPosString, goalPosString, outputSolutionFile)
                try:
                    start_time = timer()
                    subprocess.run(commandPlan.split(" "), check=True)
                    timespent = round(timer() - start_time, 2)  # Round execution time
                    times.append(timespent)

                    returncode = subprocess.run(commandVerify.split(" "), check=False).returncode
                    success = returncode == 0
                    successes += success

                    if success:
                        with open(outputSolutionFile) as f:
                            solution = [list(map(float, line.split(",")[:-1])) for line in f.readlines()[1:]]
                            solution = np.array(solution)
                            difsPos = np.abs(solution[1:] - solution[:-1])
                            cost = np.minimum(difsPos, np.abs(2*np.pi - difsPos)).sum()
                            cost = round(cost, 2)  # Round cost

                            costs.append(cost)
                            vertices.append(solution.shape[0])
                except Exception as exc:
                    print(f"Failed: {exc}")
                    times.append(5.0)
                    costs.append(-1)
                    vertices.append(-1)

                # delete outputSolutionFile
                if os.path.exists(outputSolutionFile):
                    os.remove(outputSolutionFile)

            scores.append([
                aPlanner, map_file, i,
                num_dofs,  # Number of DOFs
                startPosString,  # Rounded Start Configuration
                goalPosString,  # Rounded Goal Configuration
                round(np.mean(times), 2), round(np.std(times), 2),
                round(np.mean(costs), 2), round(np.std(costs), 2),
                round(successes / num_repeats, 2)  # Round success rate
    

            ])

    df = pd.DataFrame(scores, columns=["planner", "mapName", "problemIndex", "numDOFs", "startConfig", "goalConfig",
                                       "meanTime", "stdTime", "meanCost", "stdCost", "successRate"])
    df.to_csv(gradingCSV, index=False)

if __name__ == "__main__":
    graderMain("./planner.out", "results/results.csv")
