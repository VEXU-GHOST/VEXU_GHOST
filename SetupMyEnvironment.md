# Setting up your dev environment (Windows / macOS)

The repo now ships a Docker setup that works the same on Windows, macOS, and Linux — no WSL2 + X11 forwarding dance, no Ubuntu-version matching. Install Docker Desktop, clone the repo, and run `docker compose up -d`.

Longer-term, if you want to do robotics software on a regular basis, a native Ubuntu install (or dual-boot) will give you the best performance and hardware access. But Docker is good enough for onboarding and day-to-day ROS 2 development.

## 1) Install Docker Desktop

- **Windows:** In powershell, run `winget install -e --id Docker.DockerDesktop`
- **macOS:** <https://docs.docker.com/desktop/install/mac-install/>
  Works on Intel and Apple Silicon. On Apple Silicon the container reports `aarch64`; the stock build skips the Gazebo sim packages on that architecture.

Start Docker Desktop once after installing. Verify:

```bash
docker version
docker compose version
```

## 2) Install Git and configure SSH

You need Git on the host — the container bind-mounts your host `~/.ssh` and `~/.gitconfig` read-only, so commits/pushes from inside the container use your host identity.

- **Windows:** install [Git for Windows](https://git-scm.com/download/win). Use **PowerShell** for the SSH + clone steps below.
- **macOS:** Git comes with Xcode Command Line Tools (`xcode-select --install`) or via Homebrew (`brew install git`).

### 2.1) Generate an SSH key

In PowerShell (Windows) or Terminal (macOS):

```bash
ssh-keygen -t ed25519 -C "your_email@example.com"
```

Press Enter through all the prompts.

### 2.2) Add the key to GitHub

Print the public key:

```bash
cat ~/.ssh/id_ed25519.pub
```

Copy the entire output. On github.com:

![image](https://github.com/VEXU-GHOST/VEXU_GHOST/assets/47650195/d4107d08-13ee-4a29-ba03-7d72ea4bf5e5)

![image](https://github.com/VEXU-GHOST/VEXU_GHOST/assets/47650195/8cf1f9dc-d258-44c2-b41e-e9426eb5d103)

![image](https://github.com/VEXU-GHOST/VEXU_GHOST/assets/47650195/368667cd-da9a-4e93-b409-2d5823e26cfd)

![image](https://github.com/VEXU-GHOST/VEXU_GHOST/assets/47650195/52daac1c-d046-402d-9dd9-7d2bf45ec4eb)

Paste the key, name it something like "My Laptop" for the title, and save.

### 2.3) Configure your Git identity

```bash
git config --global user.name "Your Name"
git config --global user.email "your_email@example.com"
```

## 3) Clone the repo and launch the dev container

From PowerShell (Windows) or Terminal (macOS):

```bash
git clone git@github.com:VEXU-GHOST/VEXU_GHOST.git
cd VEXU_GHOST
git submodule update --init --recursive
git checkout develop # change develop to the branch you are working on

docker compose build              # first time only, ~10–15 min
docker compose up -d              # start the dev container + noVNC
docker compose exec vexu bash     # open a shell inside the container
```

Inside the shell:

```bash
./scripts/build.sh                # compile the ROS 2 workspace
./scripts/launch_sim.sh           # start Gazebo
```

## 4) See the GUI (RViz, Gazebo)

Open [http://localhost:8080/vnc.html](http://localhost:8080/vnc.html) in any browser on your host. That's it — no XLaunch, no XQuartz. Apps launched inside the container render into a virtual display that streams to your browser.

Stop the container when done: `docker compose stop`.
Use `docker compose down` only when you want to reset the container because it removes containers and drops writable in-container filesystem changes (for example, tools/extensions installed inside the container). State in `build/`, `install/`, `log/`, and the ccache volume persists.

## Hardware caveats

- **USB passthrough (PROS upload to a V5 brain)** is limited on Docker Desktop (macOS especially). For hardware work, run `pros upload` from a native Ubuntu install or from the Windows host directly with a native PROS install.
- **Native Ubuntu** is still the best environment for competition day and anything touching real sensors/actuators. See the "Native Ubuntu 22.04" section in the [root README](README.md) when you're ready to set that up.

## Next steps

Continue Onboarding I: [main README → Setup](README.md#setup).
