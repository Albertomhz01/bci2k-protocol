import tkinter as tk
import json
import csv
import os
import re
from datetime import datetime


# Folder where BCI2000 saves .dat files -> must match DATA_DIRECTORY.
HERE = os.path.dirname(os.path.abspath(__file__))
DATA_DIRECTORY = os.path.join(HERE, "dat_bci")


def next_run_from_disk(participant_number, session_number):
    """Get the next run number from existing .dat files.
    The files on disk are the source of truth: if R01 and R02 exist, 
    the next is R03.
    """
    subject = "P" + str(participant_number).zfill(3)
    session = str(session_number).zfill(3)
    folder = os.path.join(DATA_DIRECTORY, subject)

    if not os.path.isdir(folder):
        return 1

    pattern = subject + "S" + session + r"R(\d{2})\.dat$"
    runs = []
    for filename in os.listdir(folder):
        match = re.match(pattern, filename)
        if match:
            runs.append(int(match.group(1)))

    if not runs:
        return 1

    return max(runs) + 1


def load_experiment_log():
    if not os.path.exists("experiment_log.csv"):
        return []
    with open("experiment_log.csv", "r") as f:
        reader = csv.DictReader(f)
        return list(reader)


def find_participant(name, session_number=None):
    """Finds the participant by name...
    filters by session if provided to reset runs from R01.
    """
    records = load_experiment_log()

    matches = []
    for r in records:
        if r.get("name", "").strip().lower() != name.strip().lower():
            continue
        if session_number is not None:
            if int(r.get("session_number", 0)) != session_number:
                continue
        matches.append(r)

    if not matches:
        return None

    latest = matches[-1]
    participant_number = int(latest["participant_number"])

    if session_number is not None:
        session = session_number
    else:
        session = int(latest["session_number"])

    return {
        "participant_number": participant_number,
        "session_number":     session,
        "run_number":         next_run_from_disk(participant_number, session),
    }


def get_next_participant_number():
    records = load_experiment_log()
    numbers = [int(r["participant_number"]) for r in records if r.get("participant_number", "").isdigit()]
    return max(numbers) + 1 if numbers else 1


def save_participant_json(data):
    """Save all sessions in a single JSON file per participant."""
    json_path = f"participant_{str(data['participant_number']).zfill(3)}.json"

    if os.path.exists(json_path):
        with open(json_path, "r") as f:
            existing = json.load(f)
    else:
        existing = {
            "name":               data["name"],
            "participant_number": data["participant_number"],
            "sessions":           []
        }

    existing["sessions"].append({
        "session_number": data["session_number"],
        "run_number":     data["run_number"],
        "timestamp":      data["timestamp"],
    })

    with open(json_path, "w") as f:
        json.dump(existing, f, indent=4)

    print(f"✔  Saved → {json_path}")
    return json_path


def launch_interface():
    result = {"submitted": False, "data": None}

    root = tk.Tk()
    root.title("BCI2000 Experiment — Participant Registration")
    root.configure(bg="#1E2A3A")
    root.resizable(False, False)

    window_width, window_height = 500, 520
    screen_w = root.winfo_screenwidth()
    screen_h = root.winfo_screenheight()
    x = (screen_w - window_width) // 2
    y = (screen_h - window_height) // 2
    root.geometry(f"{window_width}x{window_height}+{x}+{y}")

    BG      = "#1E2A3A"
    PANEL   = "#26384F"
    ACCENT  = "#4A9EBF"
    TEXT    = "#E8F4F8"
    SUBTEXT = "#8BAFC4"
    ENTRY_BG= "#162030"
    BTN_BG  = "#4A9EBF"
    BTN_FG  = "#FFFFFF"
    ERROR   = "#E05C5C"
    AUTO_FG = "#5BCCA0"
    NEW_FG  = "#F0A500"

    tk.Frame(root, bg=ACCENT, height=6).pack(fill="x")

    title_frame = tk.Frame(root, bg=BG, pady=20)
    title_frame.pack(fill="x", padx=30)

    tk.Label(
        title_frame,
        text="PARTICIPANT REGISTRATION",
        font=("Consolas", 13, "bold"),
        bg=BG, fg=ACCENT,
    ).pack(anchor="w")

    status_var = tk.StringVar(value="Enter participant name to begin.")
    status_label = tk.Label(
        title_frame,
        textvariable=status_var,
        font=("Segoe UI", 9),
        bg=BG, fg=SUBTEXT
    )
    status_label.pack(anchor="w", pady=(2, 0))

    panel = tk.Frame(root, bg=PANEL, padx=25, pady=20)
    panel.pack(fill="both", expand=True, padx=20, pady=(0, 10))

    def make_field(parent, label_text, auto=False):
        row = tk.Frame(parent, bg=PANEL)
        row.pack(fill="x", pady=6)

        label_row = tk.Frame(row, bg=PANEL)
        label_row.pack(fill="x")

        tk.Label(
            label_row, text=label_text,
            font=("Segoe UI", 9, "bold"),
            bg=PANEL, fg=TEXT, anchor="w"
        ).pack(side="left")

        if auto:
            tk.Label(
                label_row, text="  AUTO",
                font=("Segoe UI", 7, "bold"),
                bg=PANEL, fg=AUTO_FG
            ).pack(side="left", padx=(4, 0))

        entry = tk.Entry(
            row,
            font=("Consolas", 11),
            bg=ENTRY_BG, fg=TEXT,
            insertbackground=ACCENT,
            relief="flat", bd=0,
            highlightthickness=1,
            highlightbackground=SUBTEXT,
            highlightcolor=ACCENT,
        )
        entry.pack(fill="x", ipady=6, pady=(3, 0))
        return entry

    name_entry        = make_field(panel, "Participant Name")
    participant_entry = make_field(panel, "Participant Number", auto=True)
    session_entry     = make_field(panel, "Session Number",     auto=True)
    run_entry         = make_field(panel, "Run Number",         auto=True)

    for entry in [participant_entry, session_entry, run_entry]:
        entry.config(state="disabled", fg=SUBTEXT)

    error_var = tk.StringVar()
    tk.Label(
        panel, textvariable=error_var,
        font=("Segoe UI", 8), bg=PANEL, fg=ERROR
    ).pack(anchor="w", pady=(4, 0))

    def set_entry(entry, value, color):
        entry.config(state="normal", fg=color)
        entry.delete(0, "end")
        entry.insert(0, str(value))

    def on_name_change(*args):
        name = name_entry.get().strip()
        if len(name) < 2:
            for entry in [participant_entry, session_entry, run_entry]:
                entry.config(state="disabled")
            status_var.set("Enter participant name to begin.")
            status_label.config(fg=SUBTEXT)
            return

        data = find_participant(name)

        if data:
            set_entry(participant_entry, data["participant_number"], AUTO_FG)
            set_entry(session_entry,     data["session_number"],     AUTO_FG)
            set_entry(run_entry,         data["run_number"],         AUTO_FG)
            participant_entry.config(state="disabled")
            run_entry.config(state="disabled")
            session_entry.config(state="normal")
            status_var.set(f"✔  Returning participant — Run {data['run_number']} detected.")
            status_label.config(fg=AUTO_FG)
        else:
            next_num = get_next_participant_number()
            set_entry(participant_entry, next_num, NEW_FG)
            set_entry(session_entry,     1,        NEW_FG)
            set_entry(run_entry,         next_run_from_disk(next_num, 1), NEW_FG)
            participant_entry.config(state="disabled")
            run_entry.config(state="disabled")
            session_entry.config(state="normal")
            status_var.set(f"★  New participant — Number {next_num} assigned.")
            status_label.config(fg=NEW_FG)

    def on_session_change(*args):
        """Recalculates the run for the selected session, restarting from R01."""
        name = name_entry.get().strip()
        session = session_entry.get().strip()

        if len(name) < 2 or not session.isdigit():
            return

        participant = participant_entry.get().strip()
        if not participant.isdigit():
            return

        run = next_run_from_disk(int(participant), int(session))
        set_entry(run_entry, run, AUTO_FG if run > 1 else NEW_FG)
        run_entry.config(state="disabled")
        status_var.set(f"Sesión {int(session)} — siguiente run: R{str(run).zfill(2)}")
        status_label.config(fg=AUTO_FG if run > 1 else NEW_FG)

    name_entry.bind("<KeyRelease>", on_name_change)
    session_entry.bind("<KeyRelease>", on_session_change)

    def on_submit():
        name        = name_entry.get().strip()
        participant = participant_entry.get().strip()
        session     = session_entry.get().strip()
        run         = run_entry.get().strip()

        if not name:
            error_var.set("⚠  Participant Name is required.")
            return
        if not participant.isdigit():
            error_var.set("⚠  Participant Number is invalid.")
            return
        if not session.isdigit():
            error_var.set("⚠  Session Number must be numeric.")
            return

        data = {
            "name":               name,
            "participant_number": int(participant),
            "session_number":     int(session),
            "run_number":         int(run),
            "timestamp":          datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        }

        # Save single JSON per participant with all sessions
        save_participant_json(data)

        # Save CSV log
        csv_path = "experiment_log.csv"
        write_header = not os.path.exists(csv_path)
        with open(csv_path, "a", newline="") as f:
            import csv as _csv
            writer = _csv.DictWriter(f, fieldnames=data.keys())
            if write_header:
                writer.writeheader()
            writer.writerow(data)

        result["submitted"] = True
        result["data"] = data
        root.destroy()

    btn_frame = tk.Frame(root, bg=BG, pady=10)
    btn_frame.pack(fill="x", padx=20)

    tk.Button(
        btn_frame,
        text="START EXPERIMENT  →",
        font=("Consolas", 10, "bold"),
        bg=BTN_BG, fg=BTN_FG,
        activebackground="#3A8EAF",
        activeforeground=BTN_FG,
        relief="flat", bd=0,
        cursor="hand2",
        padx=20, pady=10,
        command=on_submit
    ).pack(fill="x")

    root.mainloop()

    return result["data"] if result["submitted"] else None


if __name__ == "__main__":
    data = launch_interface()
    if data:
        print("\nCaptured data:")
        for k, v in data.items():
            print(f"  {k}: {v}")
    else:
        print("Cancelled.")