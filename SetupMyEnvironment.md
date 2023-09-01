### Linux Users
Requires Ubuntu 22.04.
Open a terminal and enter:
```
sudo apt-get install openssh-client git
```
Skip windows setup and continue to [SSH Setup](#ssh-setup).

### Windows Users
While you really need an Ubuntu operating system if you want to do software for robotics in the long-term (see https://itsfoss.com/install-ubuntu-1404-dual-boot-mode-windows-8-81-uefi/), if you want to try things out without committing to reconfiguring your whole computer, we have a workaround using WSL2.

WSL2 is "Windows Subsystem for Linux" and allows you to develop using Linux within your Windows OS. I _**highly**_ recommend using this option over a standard Virtual Machine, as VM's have terrible performance in my experience. We will be rendering graphics on the Windows-side using a concept called X11 Forwarding, so WSL2 ends up with substantially improved performance.

The biggest tradeoff with not having a "true" Linux installation is you will not be able to interface directly to robot hardware and will likely have trouble with USB input (Might work out on Windows 11 via usb-passthrough).

#### Install WSL2
Make sure you install **WSL2** and **Ubuntu 22.04** or you will have to redo it.

Link: https://www.omgubuntu.co.uk/how-to-install-wsl2-on-windows-10

#### Install X11 Forwarding
This is my preferred application for X11 Forwarding.
https://sourceforge.net/projects/vcxsrv/

#### Configure ~/.bashrc
```
echo "export DISPLAY=$(ip route list default | awk '{print $3}'):0" >> ~/.bashrc
echo "export LIBGL_ALWAYS_INDIRECT=0" >> ~/.bashrc
```

#### Try it out
Open VcXsrv. Default options may work, may also need to disable "Native OpenGL".
```
sudo apt-get install x11-apps
xeyes
```

This should spawn two eyes which follow your cursor! Continue to [SSH Setup](#ssh-setup).

If you have issues, come find me and I will help you out (or if you found this remotely, you can email me at jessemaxxwilson@utexas.edu).
Full disclosure, I've dual-booted for awhile and so I don't test these instructions regularly.

### SSH Setup 
If you haven't used Github on your computer before, you will need to add SSH keys. These let github recognize your computer (and this replaced using password last year).

**Generate Key:** https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent \
**Add Key:** https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account
