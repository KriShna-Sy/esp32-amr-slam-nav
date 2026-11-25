#!/usr/bin/env python3
"""
view_logs.py
Simple CLI tool to inspect and debug robot activities:
  - python3 view_logs.py           : List recent activity sessions
  - python3 view_logs.py --last    : Print the complete log of the last activity
  - python3 view_logs.py --errors  : Show only failed sessions with debug hints
  - python3 view_logs.py --tail    : Stream the live master log
  - python3 view_logs.py <id>      : View a specific session log
"""

import os
import sys
import glob

LOGS_DIR = os.path.expanduser('~/mapping_ws/logs')
SESSIONS_DIR = os.path.join(LOGS_DIR, 'sessions')
MASTER_LOG_FILE = os.path.join(LOGS_DIR, 'robot_activity.log')
LATEST_LINK = os.path.join(LOGS_DIR, 'latest.log')

# ANSI Colors
GREEN = "\033[92m"
RED = "\033[91m"
YELLOW = "\033[93m"
BLUE = "\033[94m"
CYAN = "\033[96m"
BOLD = "\033[1m"
RESET = "\033[0m"


def print_banner():
    print(f"\n{BOLD}{CYAN}=== Robot Mission & Activity Log Analyzer ==={RESET}")
    print(f"Log location: {MASTER_LOG_FILE}\n")


def list_sessions(errors_only=False):
    print_banner()
    if not os.path.exists(SESSIONS_DIR):
        print(f"No log sessions found in {SESSIONS_DIR} yet.")
        return

    session_files = sorted(glob.glob(os.path.join(SESSIONS_DIR, "*.txt")), reverse=True)
    if not session_files:
        print("No sessions recorded yet.")
        return

    print(f"{BOLD}{'SESSION ID':<32} {'DATE / TIME':<20} {'RESULT':<12} {'SUMMARY / DEBUG HINT'}{RESET}")
    print("-" * 90)

    count = 0
    for s_file in session_files[:30]:
        filename = os.path.basename(s_file).replace(".txt", "")
        # Parse result and summary
        result = "UNKNOWN"
        summary = ""
        dt_str = ""
        with open(s_file, "r", encoding="utf-8", errors="ignore") as f:
            for line in f:
                if "ACTIVITY START:" in line and not dt_str:
                    try:
                        dt_str = line.split("]")[0].strip("[")
                    except Exception:
                        pass
                if "RESULT:" in line:
                    result = line.split("RESULT:")[1].strip()
                if "Summary    :" in line and not summary:
                    summary = line.split("Summary    :")[1].strip()
                if "Explanation:" in line and not summary:
                    summary = line.split("Explanation:")[1].strip()
                if "DEBUG HINT :" in line:
                    summary = line.split("DEBUG HINT :")[1].strip()

        if errors_only and result != "FAILED":
            continue

        res_colored = (
            f"{GREEN}SUCCESS{RESET}" if result == "SUCCESS"
            else f"{RED}FAILED{RESET}" if result == "FAILED"
            else f"{YELLOW}{result}{RESET}"
        )

        short_summary = (summary[:45] + "...") if len(summary) > 48 else summary
        print(f"{filename:<32} {dt_str[:19]:<20} {res_colored:<21} {short_summary}")
        count += 1

    if count == 0 and errors_only:
        print(f"{GREEN}No failed sessions recorded! All recent runs succeeded.{RESET}")

    print("-" * 90)
    print("Commands:")
    print("  python3 view_logs.py --last     (View full log of most recent run)")
    print("  python3 view_logs.py --errors   (Filter to only failed runs)")
    print("  python3 view_logs.py <session>  (View specific session file)")
    print("  python3 view_logs.py --tail     (Live follow log output)\n")


def view_file(filepath):
    if not os.path.exists(filepath):
        print(f"{RED}File not found:{RESET} {filepath}")
        return
    print(f"\n{BOLD}{CYAN}--- Viewing: {filepath} ---{RESET}\n")
    with open(filepath, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            if "SUCCESS" in line:
                print(f"{GREEN}{line.rstrip()}{RESET}")
            elif "FAILED" in line or "ERROR" in line:
                print(f"{RED}{line.rstrip()}{RESET}")
            elif "DEBUG HINT" in line or "WARN" in line:
                print(f"{YELLOW}{line.rstrip()}{RESET}")
            elif "STEP" in line:
                print(f"{BLUE}{line.rstrip()}{RESET}")
            else:
                print(line.rstrip())
    print()


def tail_master_log():
    if not os.path.exists(MASTER_LOG_FILE):
        print(f"Master log not created yet: {MASTER_LOG_FILE}")
        return
    print(f"{BOLD}Streaming {MASTER_LOG_FILE} (Ctrl-C to stop)...{RESET}\n")
    os.system(f"tail -f -n 40 {MASTER_LOG_FILE}")


def main():
    if len(sys.argv) == 1:
        list_sessions(errors_only=False)
    elif sys.argv[1] in ("-l", "--last", "last"):
        if os.path.exists(LATEST_LINK):
            view_file(os.path.realpath(LATEST_LINK))
        else:
            files = sorted(glob.glob(os.path.join(SESSIONS_DIR, "*.txt")), reverse=True)
            if files:
                view_file(files[0])
            else:
                print("No sessions recorded yet.")
    elif sys.argv[1] in ("-e", "--errors", "errors"):
        list_sessions(errors_only=True)
    elif sys.argv[1] in ("-f", "--tail", "--follow", "tail"):
        tail_master_log()
    elif sys.argv[1] in ("-h", "--help"):
        print(__doc__)
    else:
        target = sys.argv[1]
        if not target.endswith(".txt"):
            target += ".txt"
        path = os.path.join(SESSIONS_DIR, target)
        if os.path.exists(path):
            view_file(path)
        elif os.path.exists(target):
            view_file(target)
        else:
            print(f"Session not found: {sys.argv[1]}")


if __name__ == "__main__":
    main()
