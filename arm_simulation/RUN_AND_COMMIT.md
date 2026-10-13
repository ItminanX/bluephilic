# Run it on your PC, test it, and commit it to GitHub (Windows / PowerShell)

Paths below match your setup: the project folder is `C:\Users\User\bluephilic` and Python is the scoop install you already use.

## 0. Where to keep the files
Keep them as a **subfolder of your existing repo**: `C:\Users\User\bluephilic\arm_simulation\`.
- `bluephilic` is already your git repo (the one you push to Render). One repo keeps `server.py`, the ESP32 sketches and the arm simulation together.
- Always run the simulation scripts from *inside* `arm_simulation\` (they import each other by file name).
- Do **not** put these files in the repo root: names like `designs.py` and `test_model.py` would mix with your vision files.
- Alternative: a separate repo (`git init` inside `arm_simulation\`). Choose that only if you want Render to never see it - section 6 shows a cheaper way.

## 1. Install (once)
Download `arm_simulation.zip`, then in PowerShell:
```powershell
Expand-Archive -Path "$HOME\Downloads\arm_simulation.zip" -DestinationPath "C:\Users\User\bluephilic" -Force
cd C:\Users\User\bluephilic\arm_simulation
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r requirements-sim.txt
```
- If PowerShell refuses to run `Activate.ps1`: `Set-ExecutionPolicy -Scope CurrentUser RemoteSigned`, then retry (or skip the venv and run `pip install numpy matplotlib` globally, like your other tools).
- The simulation needs only **numpy and matplotlib**. Your server's packages (fastapi, opencv...) are not needed for it.
- Do **not** add matplotlib to the repo-root `requirements.txt` - that file is what Render installs.

## 2. Test it
```powershell
python test_model.py     # expect 8 lines starting with PASS
python test_bridge.py    # expect 2 lines starting with PASS
```
Both take a few seconds. If a line says `AssertionError`, do not commit; the test name tells you what broke:

| Failing test | Usually means |
|---|---|
| `neutral_pose_matches_builder_description`, `pan_sign` | you edited `ArmParams` signs/offsets (`m3_dir`, `m3_rel_at_mean_deg`) |
| `fk_ik_roundtrip`, `unreachable_returns_none` | link lengths or limits changed; some expectation in the test needs updating on purpose |
| `camera_transform_roundtrip...` | camera parameters edited inconsistently |
| `dwell_rule`, `no_grab_while_object_keeps_moving` | controller `Config` (dwell time, tolerances) changed |

## 3. Run the simulation
```powershell
python run_sim.py          # about 25 s on the test machine; writes results\*.png and results\summary.json
python design_sweep.py     # camera bracket / search pose trade-off
python position_sweep.py   # 15 positions x 3 camera tilts (a few minutes, not timed)
python sensitivity.py      # model-error analysis (a few minutes, not timed)
```
No window opens: plots are saved as PNG files in `results\`. Open them from Explorer.

**Use your own numbers.** After measuring the real arm, edit the values in `arm_model.py` (`ArmParams`), `designs.py` (search pose, bracket position) and `arm_controller.py` (`Config`: dwell time, basket, speeds), then re-run the tests and `run_sim.py`. M3 direction: run `python calibrate_m3.py 0:-60 90:30 180:120` with your protractor readings.

## 4. Run it with your real vision model (shadow mode)
This feeds what your ESP32 + `server.py` actually produce through the controller and prints the servo numbers it *would* send. No servos needed.

Terminal 1 (as you do now), from `C:\Users\User\bluephilic`:
```powershell
uvicorn server:app --host 0.0.0.0 --port 8000
```
ESP32 streaming. Terminal 2, from `C:\Users\User\bluephilic\arm_simulation`:
```powershell
python live_bridge.py --url http://localhost:8000/latest --record logs\session1.jsonl --csv logs\session1.csv
```
It first prints where the camera must be held (about 32 cm above the floor, 18 cm ahead of the arm base, looking straight down, for the default design), then a status line every 0.5 s:
```
t=  10.0s APPROACH found=1 world=(  5.9, 23.9,  2.8) M1=104 M2= 47 M3= 58 S1= 30
```
- `M1=90` means straight ahead (M1 servo number = pan + 90). `world=` is the object position in cm on the floor frame (x right, y forward, z up).
- Hold the ESP32 on a stand at the printed pose, put a blue object in view, and keep it still for 10 s: the state goes TRACK -> APPROACH -> DESCEND -> CLOSE -> TRANSFER -> RELEASE -> HOME. It repeats afterwards because nothing removes the object.
- `--record` saves the raw JSON so you can replay it later without hardware: `python live_bridge.py --replay logs\session1.jsonl`.
- No hardware demo: `python make_sample_log.py` then `python live_bridge.py --replay sample_logs\synthetic_still_object.jsonl`.
- Limits: the camera does not move in this mode, so it checks units, filtering, the 10 s rule and the state sequence on real data, not arm motion or grasp accuracy.
- "waiting: server has no frames yet" = the ESP32 is not streaming; "cannot reach ..." = uvicorn is not running or the URL/port is wrong.

## 5. Before the first commit: secrets and ignore rules
1. **Your ESP32 sketch contains your WiFi name and password in plain text.** Do not push that to GitHub as is. In the sketch folder, create `secrets.h`:
```cpp
// secrets.h  (NOT committed)
const char* WIFI_SSID = "your-wifi-name";
const char* WIFI_PASS = "your-wifi-password";
```
   In the `.ino`, delete the two `const char* WIFI_...` lines and add `#include "secrets.h"` near the top. Also commit a copy named `secrets.example.h` with fake values so others know the format. `secrets.h` is already in `.gitignore`.
2. **Check whether the password is already in your history** (replace the quoted text with your actual password):
```powershell
cd C:\Users\User\bluephilic
git log --all --oneline -S"your-wifi-password"
```
   Any output means it was committed before. If that repo is public (or ever was), change the WiFi password - rewriting history does not undo a public push.
3. Add ignore rules to the **repo root** before using `git add .` (otherwise `.venv\` gets committed):
```powershell
@"
.venv/
__pycache__/
*.pyc
logs/
secrets.h
"@ | Add-Content .gitignore
```
   Keep `calibration.npz` tracked: `server.py` loads it and Render needs it.
4. Windows line endings: `git config --global core.autocrlf true` (LF/CRLF warnings are harmless).

## 6. Commit and push
Check the situation first:
```powershell
cd C:\Users\User\bluephilic
git status
git branch --show-current     # main or master - use that name below
git remote -v                 # should list your GitHub URL
```
**Work on a branch** so Render does not redeploy until you are ready:
```powershell
git checkout -b arm-simulation
git add .gitignore arm_simulation
git status                    # read the list: no .venv, no __pycache__, no secrets.h
git commit -m "Add arm kinematics and closed-loop simulation with tests"
git push -u origin arm-simulation
```
Then on GitHub open a pull request (or merge locally: `git checkout main; git merge arm-simulation; git push`). Pushing to `main` can trigger a Render deploy if auto-deploy is on. It is harmless for the server, but you can avoid it: in the Render service settings look for build filters / ignored paths (the label may differ) and add `arm_simulation/**`.

**If there is no GitHub remote yet:** create a new repository on github.com (choose Private, do not tick "add a README"), then:
```powershell
git remote add origin https://github.com/YOUR_USERNAME/bluephilic.git
git branch -M main
git push -u origin main
```
The first push opens a browser sign-in (Git Credential Manager). If you use the GitHub CLI instead: `gh auth login`.

**Optional - run the tests on every push (GitHub Actions):**
```powershell
New-Item -ItemType Directory -Force .github\workflows | Out-Null
Copy-Item arm_simulation\github_workflow\arm-sim-tests.yml .github\workflows\
git add .github
git commit -m "Run simulation tests in CI"
git push
```
It runs the same two commands that pass above; I could not run GitHub's servers from here, so check the Actions tab after the first push.

## 7. Habits that keep the project documented and reproducible
- Before each commit: `python test_model.py; python test_bridge.py`.
- Commit code and generated results separately (`git add *.py` then a second commit for `results\`), with messages that say what changed and why ("camera bracket 7.6 -> 13 cm: position_sweep 7/15 -> 13/15").
- Tag each phase exit: `git tag -a sim-v1 -m "kinematics + closed loop verified"; git push --tags`.
- Record every real measurement (date, how measured) in `docs\01_parameter_register.md` (layout in `GUIDE.md` section 5) and update `ArmParams` in the same commit.
- Keep your recorded `logs\*.jsonl` sessions for anything you want to reproduce: copy them to a tracked folder (for example `sample_logs\`) because `logs\` is ignored.

## 8. Quick problems
| Symptom | Fix |
|---|---|
| `python` not found | use the scoop Python you run uvicorn with; `py -3` also works |
| `ModuleNotFoundError: matplotlib` | the venv is not active, or run `pip install -r requirements-sim.txt` |
| `ModuleNotFoundError: arm_model` | you are not inside `arm_simulation\` |
| `git add .` is very slow / huge | `.venv\` is not ignored - add the rules from section 5, then `git rm -r --cached .venv` |
| `git push` rejected | `git pull --rebase origin <branch>` then push again |
| Render redeployed after the push | expected on `main`; use the branch flow or the ignored-paths setting |
