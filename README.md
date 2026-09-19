# BCI2000 Experimental Protocol

This repository contains the Python scripts and configuration files used to run the BCI2000 experimental protocol with the **g.HIamp EEG amplifier** and the pressure-control system.

## Requirements

Before running the experiment, the following software is required:

* Python
* BCI2000
* g.HIamp drivers
* Python packages listed in `requirements.txt`

This project currently uses:

```text
BCI2000 v3.6.beta.R7385
```

## 1. Clone the Repository

Clone the repository and enter the project directory:

```bash
git clone <repository-url>
cd bci2k-protocol
```

## 2. Install BCI2000

Download and install **BCI2000 v3.6.beta.R7385**.

Place the BCI2000 folder **inside the repository folder**.

The project structure should look approximately like this:

```text
bci2k-protocol/
│
├── BCI2000/
│   └── BCI2000 v3.6.beta.R7385/
│       └── BCI2000.x64/
│           └── prog/
│
├── Parameters/
├── Participants/
├── arduino_pressure_controller/
├── dat_bci/
├── experiment_controller_cruz.py
├── participant_interface.py
├── bci_setup_cruz_con_ruta.py
├── prebasal_controller.py
├── requirements.txt
└── README.md
```

> **Important:** The BCI2000 installation itself should generally not be committed to the Git repository.

## 3. Install the g.HIamp Drivers

The **g.HIamp drivers must be installed on the computer** before the experiment can communicate with the EEG amplifier.

Install the appropriate g.tec/g.HIamp drivers for the system and verify that the computer recognizes the amplifier before attempting to run the experiment.

BCI2000 will not be able to communicate with the g.HIamp hardware correctly if the required drivers are missing.

## 4. Create a Python Virtual Environment

From the repository directory, create a virtual environment:

```bash
python -m venv .venv
```

Activate it on Windows:

```bash
.venv\Scripts\activate
```

Once activated, install the Python dependencies:

```bash
python -m pip install --upgrade pip
pip install -r requirements.txt
```

## 5. Configure the BCI2000 Python Path

Python must be able to locate the BCI2000 `prog` directory.

Inside the virtual environment's `site-packages` directory, create a file named:

```text
bci2000.pth
```

For a standard Windows virtual environment, `site-packages` will normally be located at:

```text
bci2k-protocol\.venv\Lib\site-packages\
```

Therefore, the file should be:

```text
bci2k-protocol\.venv\Lib\site-packages\bci2000.pth
```

Inside `bci2000.pth`, add the **absolute path** to the BCI2000 `prog` directory.

For example:

```text
C:\Users\<USERNAME>\<PATH>\bci2k-protocol\BCI2000\BCI2000 v3.6.beta.R7385\BCI2000.x64\prog
```

The important part is that the path ends in:

```text
BCI2000\BCI2000 v3.6.beta.R7385\BCI2000.x64\prog
```

Do **not** simply copy another user's absolute path. The path must correspond to the location of the repository on the current computer.

### Verify the configuration

With the virtual environment activated, run:

```bash
python -c "import sys; print('\n'.join(sys.path))"
```

The BCI2000 `prog` directory should appear in the output.

You can also locate the active `site-packages` directory with:

```bash
python -m site
```

## 6. Final Project Structure

After completing the setup, the relevant structure should resemble:

```text
bci2k-protocol/
│
├── .venv/
│   └── Lib/
│       └── site-packages/
│           └── bci2000.pth
│
├── BCI2000/
│   └── BCI2000 v3.6.beta.R7385/
│       └── BCI2000.x64/
│           └── prog/
│
├── Parameters/
├── Participants/
├── arduino_pressure_controller/
├── dat_bci/
├── experiment_controller_cruz.py
├── participant_interface.py
├── bci_setup_cruz_con_ruta.py
├── prebasal_controller.py
├── requirements.txt
└── README.md
```

## 7. Before Running the Experiment

Before starting a recording session, verify that:

* BCI2000 is installed inside the repository directory.
* The g.HIamp drivers are installed.
* The g.HIamp amplifier is connected and recognized by the computer.
* The Python virtual environment is activated.
* All packages from `requirements.txt` are installed.
* `bci2000.pth` exists inside the environment's `site-packages`.
* `bci2000.pth` points to the correct BCI2000 `prog` directory.

Once these requirements are satisfied, the Python scripts should be able to locate and interact with the required BCI2000 components.
