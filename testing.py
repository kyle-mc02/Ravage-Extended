import argparse
import subprocess

# Fault categories
BIAS_FAULTS = [
    "drift-neg",
    "drift-pos",
]

NON_BIAS_FAULTS = [
    "precision_damage",
    "stuck",
    "short_circuit",
]

T_VALUES = [2, 5, 10]   # your combinations


def run_tests(faults, sensor):
    for fault in faults:
        for t in T_VALUES:
            for _ in range(1):
                process = subprocess.Popen(
                    [
                        "python", "ravage.py",
                        "-s", "ArduPilot",
                        "-a", sensor,
                        "-t", str(t),
                        "-i", "3",
                        "-f", fault
                    ],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True
                )

                stdout, stderr = process.communicate()
                print(f"[FAULT={fault}]  -t {t}")
                print("STDOUT:", stdout.strip())
                print("STDERR:", stderr.strip())
                print("-" * 60)


def parse_args():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--mode",
        choices=["bias", "nonbias"],
        required=True,
        help="Choose which fault category to test"
    )

    parser.add_argument(
        "--sensor",
        choices=["GPS", "GYRO", "MAG", "OPTICAL_FLOW", "ACCELEROMETER"],
        default="GPS",
        required=False,
        help="Choose which fault category to test"
    )

    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()

    if args.mode == "bias":
        faults_to_run = BIAS_FAULTS
    else:
        faults_to_run = NON_BIAS_FAULTS

    run_tests(faults_to_run, args.sensor)
