# Connecting to the Pi (bolzpi)

A quick-reference guide for getting `bolzpi` onto a hotspot, connecting over SSH, and activating the Python virtual environment.

---

## Part 1 — Connect the Pi to a Hotspot

> **Note:** This assumes the hotspot's SSID and password are already saved as a known network on the Pi. If they are not saved yet, the Pi cannot join automatically — it needs to be added first (see setup notes).

### Step 1 — Make the phone's hotspot discoverable

On the phone that will provide internet:

- Turn on Personal Hotspot / Mobile Hotspot in phone settings.
- Keep the hotspot screen open/active for a minute or two while the Pi searches for it — some phones stop broadcasting if the settings screen is backgrounded too quickly.
- Confirm the SSID (network name) and password match what was already saved on the Pi.

### Step 2 — Power on the Pi

Plug in the Pi and wait about 30–60 seconds for it to fully boot and attempt to join the known hotspot automatically.

### Step 3 — Confirm the Pi joined the network

From the same phone (or another device on the same hotspot), check the list of connected devices in the hotspot settings. Look for a device named `bolzpi`.

> **Troubleshooting:** If it doesn't appear after a minute, double-check the hotspot is still broadcasting and the phone hasn't gone to sleep.

---

## Part 2 — SSH into the Pi

### Step 1 — Open a terminal

On your Mac, open Terminal.

### Step 2 — Connect over SSH

```bash
ssh pi@bolzpi.local
```

### Step 3 — Enter the password

When prompted for a password, type:

```
*****
```

> **Note:** The terminal will not show any characters as you type the password — this is normal. Just type it and press Enter.

### Step 4 — Confirm you're connected

Your terminal prompt should now change to something like:

```
pi@bolzpi:~ $
```

> **Troubleshooting:** If you instead see "Operation timed out" or "Could not resolve hostname", the Pi and your Mac are likely not on the same network yet. Go back to Part 1 and confirm the Pi has joined the hotspot before retrying.

---

## Part 3 — Navigate to the Project and Activate the Virtual Environment

### Step 1 — Move into the project directory

```bash
cd ~/drone_project
```

### Step 2 — Activate the virtual environment

```bash
source ~/mavenv/bin/activate
```

### Step 3 — Confirm it's active

Your prompt should now show `(mavenv)` at the start, like this:

```
(mavenv) pi@bolzpi:~/drone_project $
```

> **Note:** Once active, `python3` and `pip` commands automatically use the packages installed in this environment (pymavlink, pyserial, etc.) — no extra flags needed.

### Step 4 — When you're done

To leave the virtual environment:

```bash
deactivate
```

---

## Part 4 — Get the Latest Files from GitHub

The project repo is public, so no GitHub login is needed — just pull the latest changes.

### If the repo is already cloned on the Pi

```bash
cd ~/UAS_PI5Drone_Autonomous
git pull
```

This updates your local copy on the Pi with anything new pushed to GitHub, including this guide.

### If the repo is not cloned on the Pi yet

```bash
git clone https://github.com/dchavan2192/UAS_PI5Drone_Autonomous.git
cd UAS_PI5Drone_Autonomous
```

> **Note:** Since the repo is public, no username, password, or token is required for `git pull` or `git clone`.

---

## Quick Reference

```bash
ssh pi@bolzpi.local
# password: 

cd ~/drone_project
source ~/mavenv/bin/activate

# Get latest files from GitHub
cd ~/UAS_PI5Drone_Autonomous
git pull
```
