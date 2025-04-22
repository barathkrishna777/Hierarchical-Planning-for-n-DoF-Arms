import numpy as np
import argparse
import subprocess
import pandas as pd
from timeit import default_timer as timer
import random
import os
import math # Added math import

# ---------- DH Forward Kinematics (Needed for Z check) ----------
# Calculate FK relative to base frame origin (0,0,0)
# Matches the core calculation within C++ IsValidArmConfiguration

def get_transformation_matrix(theta, d, a, alpha):
    """Calculates the homogenous transformation matrix for DH parameters."""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    # Standard DH Transformation Matrix
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0,       sa,       ca,     d],
        [0,        0,        0,     1]
    ])

def forward_kinematics_ur5(angles, link_length=250):
    """
    Calculates the positions of all joints relative to the base frame origin (0,0,0).

    Args:
        angles (list or np.array): List of 6 joint angles in radians.
        link_length (float): The length used for scaling DH 'a' parameters.

    Returns:
        np.array: An array of [x, y, z] positions for each joint frame origin,
                  starting with the base frame origin (0,0,0).
    """
    # DH parameters matching C++ usage from utils.h
    # LINKLENGTH_CELLS is 100 in C++ files
    d_params = [1.0, 0.0, 0.0, 1.0, 0.0, 1.0] # Raw values like C++
    alpha_params = [math.pi / 2, 0.0, math.pi / 2, -math.pi / 2, math.pi / 2, 0.0]
    a_params = [0.0, link_length, link_length, 0.0, 0.0, 0.0] # Scaled by link_length like C++

    # Start kinematics from Identity (origin of base frame)
    T_cumulative = np.identity(4)
    # Store positions, starting with base frame origin
    positions = [T_cumulative[:3, 3]]

    if len(angles) != 6:
         print(f"Warning: FK Expected 6 angles, got {len(angles)}. Using first 6 if available.")

    # Iterate through each joint transformation
    for i in range(min(len(angles), 6)):
        current_d = d_params[i]
        current_a = a_params[i]
        current_alpha = alpha_params[i]
        current_theta = angles[i]

        # Calculate the transformation for the current joint
        T_i = get_transformation_matrix(current_theta, current_d, current_a, current_alpha)

        # Post-multiply to get the transform to the end of the current link
        # T_cumulative represents the transform from base to the *start* of link i
        # T_i represents the transform from the start of link i to the end of link i
        T_cumulative = T_cumulative @ T_i

        # Store the position of the frame at the end of the current link
        positions.append(T_cumulative[:3, 3])

    return np.array(positions)

# ---------- End DH Forward Kinematics ----------


def convertPIs(aString):
    """Converts a string with 'pi' and expressions to rounded floats."""
    if not isinstance(aString, str): # Handle if input is already list/array
        aString = ",".join(map(str, aString))
    if aString.endswith(","):
        aString = aString[:-1] # Remove trailing comma if present
    aString = aString.replace("pi", str(math.pi)) # Replace 'pi' with its value
    try:
        vecOfStrings = aString.split(",")
        # Evaluate each expression and round to 2 decimal places
        return [str(round(eval(anExpression), 2)) for anExpression in vecOfStrings]
    except Exception as e:
        print(f"Error evaluating expression in convertPIs: '{aString}' -> {e}")
        return [] # Return empty list on error


def generate_random_configuration(num_dofs, map_file):
    """
    Generates a random, valid configuration, attempting to find
    one where the end-effector Z coordinate (in world frame) is close to 0.

    Args:
        num_dofs (int): Number of degrees of freedom (should be 6).
        map_file (str): Path to the map file.

    Returns:
        str: A comma-separated string of valid joint angles, or raises RuntimeError.

    Raises:
        RuntimeError: If no suitable configuration is found within max_attempts.
    """
    attempts = 0
    # --- *** REDUCE Z Tolerance significantly *** ---
    # Original was 10. Trying 1.0. Adjust if needed (smaller is stricter).
    z_tolerance = 5.0
    # --- *** End Tolerance Change *** ---
    max_attempts = 10000 # Increase max attempts as it will be harder

    # Assumed Z offset of the robot base in world coordinates (from C++ utils.h)
    world_base_z = 0.0
    target_world_z = 0.0 # The desired world Z coordinate

    print(f"Searching for valid config with EE World Z near {target_world_z:.2f} +/- {z_tolerance:.2f}...")

    while attempts < max_attempts:
        attempts += 1
        # Generate random angles [0, 2*pi)
        config_angles = np.random.uniform(0, 2 * np.pi, num_dofs)

        # Calculate FK positions relative to base frame origin
        # Use link_length = 100 based on LINKLENGTH_CELLS define
        positions_relative = forward_kinematics_ur5(config_angles, link_length=250)
        # Get the position of the end-effector frame relative to the base frame
        ee_pos_relative = positions_relative[-1] # Last position is EE [x, y, z]

        # Estimate world Z coordinate by adding base Z offset
        # Assumes config_checker uses the same base offset logic as utils.h
        ee_z_world_estimated = ee_pos_relative[2] + world_base_z # Z is index 2

        # *** Check if estimated world Z is close to the target (0.0) ***
        if abs(ee_z_world_estimated - target_world_z) < z_tolerance:
            # Only print check message if Z condition met, to reduce noise
            print(f"Attempt {attempts}: Config has EE Z ~ {ee_z_world_estimated:.2f}. Checking validity...")
            # Format angles to string with 2 decimal places for the checker
            config_str = ",".join(map(lambda x: f"{x:.2f}", config_angles))
            # Call external C++ checker
            if is_valid_configuration(config_str, num_dofs, map_file):
                print(f"Valid config found with EE Z ~ {ee_z_world_estimated:.2f} after {attempts} attempts: {config_str}")
                return config_str
            # else: # Optional: print if Z was okay but config invalid
            #     print(f"Attempt {attempts}: Config with EE Z ~ {ee_z_world_estimated:.2f} was invalid.")

        # Print progress periodically
        elif attempts % 500 == 0:
             print(f"Attempt {attempts}: Still searching...")


    print(f"Warning: Failed to find a valid configuration with EE Z close to {target_world_z:.2f} +/- {z_tolerance:.2f} after {max_attempts} attempts.")
    # Raise error if no configuration is found
    raise RuntimeError(f"Could not find valid config with EE Z near {target_world_z:.2f} +/- {z_tolerance:.2f} after {max_attempts} attempts")


def generate_start_and_goal(num_dofs,
                            map_file,
                            z_tolerance=5.0,
                            max_attempts=10000,
                            link_length=250,
                            world_base_z=0.0,
                            target_world_z=0.0):
    """
    1) Sample up to max_attempts for a valid *start* config.
    2) Once found, sample up to max_attempts for a valid *goal* config.
    A candidate is “valid” if:
      a) its EE world‐Z is within ±z_tolerance of target_world_z,
      b) `is_valid_configuration(…)` returns True.
    Returns (start_cfg_str, goal_cfg_str) or raises RuntimeError.
    """
    def sample_one(role):
        for attempt in range(1, max_attempts+1):
            # 1) random angles
            angles = np.random.uniform(0, 2*np.pi, num_dofs)

            # 2) compute EE z via your FK
            pos = forward_kinematics_ur5(angles, link_length=link_length)
            ee_z = pos[-1, 2] + world_base_z
            if abs(ee_z - target_world_z) > z_tolerance:
                continue

            # 3) full C++ check
            cfg_str = ",".join(f"{a:.2f}" for a in angles)
            if is_valid_configuration(cfg_str, num_dofs, map_file):
                print(f"{role.capitalize()} found after {attempt} tries: Z≈{ee_z:.2f}, cfg={cfg_str}")
                return cfg_str

            # (optional) print progress every 500 iters
            if attempt % 500 == 0:
                print(f"  {role} sampling: {attempt} attempts so far…")

        raise RuntimeError(f"Could not find valid {role} config after {max_attempts} attempts")

    start_cfg = sample_one("start")
    goal_cfg  = sample_one("goal")
    return start_cfg, goal_cfg


def is_valid_configuration(config, num_dofs, map_file):
    """Checks configuration validity using the external C++ checker."""
    # Ensure config is a comma-separated string
    if isinstance(config, (list, np.ndarray)):
         config = ",".join(map(str, config))

    # Construct the command
    command = f"./config_checker.out {map_file} {num_dofs} {config}"
    # Run the command
    try:
        # Use a timeout to prevent hangs
        result = subprocess.run(command.split(" "), check=False, capture_output=True, text=True, timeout=10)
        # Debugging output (optional)
        # if result.returncode != 0:
        #    print(f"Config Checker failed (Code {result.returncode}) for: {config}")
        #    print(f"  Stderr: {result.stderr.strip()}")
        #    print(f"  Stdout: {result.stdout.strip()}")
        return result.returncode == 0 # Return True if exit code is 0 (valid)
    except subprocess.TimeoutExpired:
        print(f"Warning: config_checker timed out for config {config}")
        return False
    except FileNotFoundError:
        print(f"Error: config_checker.out not found. Make sure it's compiled and in the current directory.")
        # Exit or raise a more specific error if the checker is essential
        raise
    except Exception as e:
        print(f"Error running config_checker for config {config}: {e}")
        return False


def graderMain(executablePath, gradingCSV):
    """Runs the grading process: generates test cases, runs planner, verifies, saves results."""
    random.seed(42)
    np.random.seed(42)
    # Define maps and parameters
    maps = ["robot_maps/map_corridor.txt"] # Use your desired map(s)
    num_problems = 5 # Number of start/goal pairs to generate
    num_repeats = 1 # Number of times to run the planner for each problem
    planners = [0] # List of planner IDs to test (e.g., [0, 1, 2])
    scores = [] # To store results for CSV

    # Generate Test Cases
    test_cases = []
    print("--- Generating Test Cases ---")
    for i in range(num_problems):
        print(f"\nGenerating Test Case {i+1}/{num_problems}...")
        map_file = random.choice(maps)
        num_dofs = 6 # Assuming 6 DoF
        try:
            # Generate start and goal configurations using the updated function
            # print("Generating start configuration...")
            # start_config_str = generate_random_configuration(num_dofs, map_file)
            # print("Generating goal configuration...")
            # goal_config_str = generate_random_configuration(num_dofs, map_file)
            start_config_str, goal_config_str = generate_start_and_goal(num_dofs, map_file)

            # Ensure configurations were generated successfully
            if start_config_str is None or goal_config_str is None:
                 print(f"Skipping test case {i+1} due to generation failure.")
                 continue # Skip if generation failed

            test_cases.append((map_file, num_dofs, start_config_str, goal_config_str))
            print(f"Generated test case {i+1}: Map: {map_file}")
            print(f"  Start Config (angles): {start_config_str}")
            print(f"  Goal Config (angles): {goal_config_str}")

        except RuntimeError as e:
             # Handle error if generate_random_configuration fails after max attempts
             print(f"Error generating test case {i+1}: {e}")
             print("Skipping this test case.")
             continue
        except FileNotFoundError:
             print("Error: config_checker.out not found during test case generation. Aborting.")
             return # Stop if checker is missing


    # Run Planner Tests
    print(f"\n--- Running Planner Tests ({len(test_cases)} cases) ---")
    for aPlanner in planners:
        print(f"\nTesting Planner ID: {aPlanner}")
        for i, (map_file, num_dofs, start_config_str, goal_config_str) in enumerate(test_cases):
            print(f"\nRunning Test Case {i+1}/{len(test_cases)} (Map: {map_file})...")
            # Define output file for the planner solution
            outputSolutionFile = f"tmp_planner_{aPlanner}_case_{i+1}.txt"

            # Lists to store results for this test case across repeats
            times, costs, vertices, successes = [], [], [], 0

            for r in range(num_repeats):
                print(f"  Repeat {r+1}/{num_repeats}")
                # Construct commands for planner and verifier
                commandPlan = f"{executablePath} {map_file} {num_dofs} {start_config_str} {goal_config_str} {aPlanner} {outputSolutionFile}"
                commandVerify = f"./verifier.out {map_file} {num_dofs} {start_config_str} {goal_config_str} {outputSolutionFile}"

                try:
                    # --- Run Planner ---
                    start_time = timer()
                    print(f"    Executing planner: {commandPlan}")
                    # Use check=True to raise error on non-zero exit code
                    subprocess.run(commandPlan.split(" "), check=True, timeout=120) # Increased timeout
                    timespent = round(timer() - start_time, 2)
                    times.append(timespent)
                    print(f"    Planner executed in {timespent}s")

                    # --- Run Verifier ---
                    print(f"    Executing verifier: {commandVerify}")
                    # Use check=False for verifier, just check return code
                    verify_result = subprocess.run(commandVerify.split(" "), check=False, capture_output=True, text=True, timeout=30)
                    returncode = verify_result.returncode
                    success = returncode == 0
                    successes += success
                    print(f"    Verifier result: {'Success' if success else 'Fail'} (Code: {returncode})")
                    # if not success:
                    #      print(f"      Verifier Stderr: {verify_result.stderr.strip()}")
                    #      print(f"      Verifier Stdout: {verify_result.stdout.strip()}")


                    # --- Process Solution File if Verification Succeeded ---
                    if success:
                        try:
                            with open(outputSolutionFile) as f:
                                lines = f.readlines()
                                # Expecting header + path points
                                if len(lines) > 1 :
                                    # Parse solution, skip header (line 0), handle potential trailing commas
                                    solution = [list(map(float, line.strip().rstrip(',').split(",")))
                                                for line in lines[1:] if line.strip()]
                                    solution = np.array(solution)

                                    # Validate shape
                                    if solution.ndim == 2 and solution.shape[0] >= 1 and solution.shape[1] == num_dofs:
                                        # Calculate cost (Euclidean distance in C-space using angular difference)
                                        if solution.shape[0] > 1: # Need at least 2 points for cost
                                            difs = solution[1:] - solution[:-1]
                                            # Handle angle wrap-around for distance calculation
                                            difs = np.minimum(np.abs(difs), 2 * np.pi - np.abs(difs))
                                            # Sum of Euclidean distances between consecutive points in C-space
                                            step_lengths = np.sqrt(np.sum(difs**2, axis=1))
                                            cost = np.sum(step_lengths)
                                        else: # Path has only one point (start=goal?)
                                            cost = 0.0

                                        cost = round(cost, 2)
                                        costs.append(cost)
                                        # Number of waypoints including start
                                        vertices.append(solution.shape[0])
                                        print(f"    Calculated Cost: {cost}, Vertices: {solution.shape[0]}")
                                    else:
                                         print(f"    Warning: Could not parse solution or invalid shape in {outputSolutionFile}. Shape: {solution.shape if isinstance(solution, np.ndarray) else 'N/A'}")
                                         costs.append(-1) # Indicate failure
                                         vertices.append(-1)
                                else:
                                    print(f"    Warning: Solution file {outputSolutionFile} has insufficient points (Lines: {len(lines)}).")
                                    costs.append(-1)
                                    vertices.append(len(lines) -1 if len(lines)>0 else 0) # Count points found

                        except FileNotFoundError:
                             print(f"    Error: Solution file {outputSolutionFile} not found after planning.")
                             costs.append(-1)
                             vertices.append(-1)
                        except Exception as exc:
                             print(f"    Error processing solution file {outputSolutionFile}: {exc}")
                             costs.append(-1)
                             vertices.append(-1)
                    else:
                         # If verification failed
                         costs.append(-1)
                         vertices.append(-1)

                # --- Handle execution errors ---
                except subprocess.TimeoutExpired as exc:
                    print(f"    Failed: {exc.cmd} timed out after {exc.timeout}s.")
                    times.append(exc.timeout) # Record timeout duration
                    costs.append(-1)
                    vertices.append(-1)
                    # Success already 0 if timeout

                except subprocess.CalledProcessError as exc:
                    # Planner returned non-zero exit code
                    print(f"    Failed: Planner returned non-zero exit code {exc.returncode}.")
                    print(f"      Command: {' '.join(exc.cmd)}")
                    # Capture time until failure if possible
                    try: timespent_fail = timer() - start_time
                    except NameError: timespent_fail = -1 # If timer didn't start
                    times.append(round(timespent_fail, 2) if timespent_fail != -1 else 120.0) # Record time or timeout
                    costs.append(-1)
                    vertices.append(-1)
                    # Success already 0

                except FileNotFoundError as exc:
                     print(f"    Failed: Executable not found: {exc.filename}")
                     # Stop grading if executables are missing
                     raise
                except Exception as exc:
                    print(f"    Failed with unexpected error: {exc}")
                    try: timespent_fail = timer() - start_time
                    except NameError: timespent_fail = -1
                    times.append(round(timespent_fail, 2) if timespent_fail != -1 else 120.0)
                    costs.append(-1)
                    vertices.append(-1)
                    # Success already 0

                # --- Cleanup ---
                if os.path.exists(outputSolutionFile):
                    try:
                        os.remove(outputSolutionFile)
                    except OSError as e:
                        print(f"    Warning: Could not remove temp file {outputSolutionFile}: {e}")

            # --- Aggregate results for this test case ---
            mean_time = round(np.mean(times), 2) if times else -1.0
            std_time = round(np.std(times), 2) if len(times) > 1 else 0.0
            # Calculate stats only on successful runs where cost is valid
            valid_costs = [c for c in costs if c != -1]
            mean_cost = round(np.mean(valid_costs), 2) if valid_costs else -1.0
            std_cost = round(np.std(valid_costs), 2) if len(valid_costs) > 1 else 0.0
            # Calculate success rate
            success_rate = round(successes / num_repeats, 2) if num_repeats > 0 else 0.0

            scores.append([
                aPlanner, map_file, i + 1, # Use 1-based problem index
                num_dofs,
                start_config_str, # Store the generated config string
                goal_config_str,  # Store the generated config string
                mean_time, std_time,
                mean_cost, std_cost,
                success_rate
            ])
            print(f"  Test Case {i+1} Aggregated Results: Time={mean_time:.2f}±{std_time:.2f}, Cost={mean_cost:.2f}±{std_cost:.2f}, Success={success_rate:.2f}")


    # --- Save Results ---
    df = pd.DataFrame(scores, columns=["planner", "mapName", "problemIndex", "numDOFs", "startConfig", "goalConfig",
                                       "meanTime", "stdTime", "meanCost", "stdCost", "successRate"])
    # Ensure results directory exists
    results_dir = os.path.dirname(gradingCSV)
    if results_dir and not os.path.exists(results_dir):
         print(f"Creating results directory: {results_dir}")
         os.makedirs(results_dir)

    df.to_csv(gradingCSV, index=False)
    print(f"\nResults saved to {gradingCSV}")

# --- Main Execution Guard ---
if __name__ == "__main__":
    # Define planner executable path and output CSV path
    planner_executable = "./planner.out" # Make sure this exists and is executable
    results_csv_path = "results/results_ee_z_strict.csv" # Changed output filename

    # Check if planner executable exists
    if not os.path.isfile(planner_executable):
         print(f"Error: Planner executable not found at '{planner_executable}'")
         print("Please compile the planner (e.g., using 'make') and ensure it's in the correct location.")
    elif not os.access(planner_executable, os.X_OK):
         print(f"Error: Planner executable at '{planner_executable}' is not executable.")
         print("Please grant execute permissions (e.g., 'chmod +x planner.out').")
    else:
        # Run the grading process
        graderMain(planner_executable, results_csv_path)

