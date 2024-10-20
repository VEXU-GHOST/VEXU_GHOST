# Windows Users
While you really need an Ubuntu operating system if you want to do software for robotics in the long-term (see https://itsfoss.com/install-ubuntu-1404-dual-boot-mode-windows-8-81-uefi/), if you want to try things out without committing to reconfiguring your whole computer, we have a workaround using WSL2.

WSL2 is "Windows Subsystem for Linux" and allows you to develop using Linux within your Windows OS. I _**highly**_ recommend using this option over a standard Virtual Machine, as VM's have terrible performance in my experience. We will be rendering graphics on the Windows-side using a concept called X11 Forwarding, so WSL2 ends up with substantially improved performance.

The biggest tradeoff with not having a "true" Linux installation is you will not be able to interface directly to robot hardware and will likely have trouble with USB input (Might work out on Windows 11 via usb-passthrough).

## 1) Install WSL2
Make sure you install **WSL2** and **Ubuntu 22.04** or you will have to redo it.

Link: https://www.omgubuntu.co.uk/how-to-install-wsl2-on-windows-10

## 2) Install X11 Forwarding
X11 Forwarding allows you to send graphics from your WSL Installation to your windows computer.
### 2.1) Install XLaunch
https://sourceforge.net/projects/vcxsrv/
You should now have this application available via the start menu or desktop.

![image](https://github.com/VEXU-GHOST/VEXU_GHOST/assets/47650195/eb835791-7247-4fad-a99e-40ed7254cd8d)
### 2.2) Configure XLaunch
Open XLaunch for the first time and match the **following configuration exactly.** Click next after each window.

![image](https://github.com/VEXU-GHOST/VEXU_GHOST/assets/47650195/bc51573e-c684-4891-8d76-d5b8f0c5df8d)

![image](https://github.com/VEXU-GHOST/VEXU_GHOST/assets/47650195/bc8b48d8-4597-4114-835b-86b0dba355e6)

**This next one is different!**

![image](https://github.com/VEXU-GHOST/VEXU_GHOST/assets/47650195/7c911e71-5cfa-4836-b227-b5b8fabd70f4)

Finally, we will save this configuration to your desktop for you to use next time. Delete the original XLaunch shortcut afterwards so you don't get confused.

![image](https://github.com/VEXU-GHOST/VEXU_GHOST/assets/47650195/defabcc3-5ee7-433f-ac44-02d13e7da7c3)

If XLaunch is running in the background, you should see this icon in the bottom rightof Windows. You will need to restart it everytime you restart your computer.

![image](https://github.com/VEXU-GHOST/VEXU_GHOST/assets/47650195/316ec88a-5ece-44f4-93f4-a2c45bd7c9b5)


## 3) Configure ~/.bashrc
In a WSL2 Terminal, run the following lines (separately):
```
echo "export DISPLAY=$(ip route list default | awk '{print $3}'):0" >> ~/.bashrc
```
```
echo "export LIBGL_ALWAYS_INDIRECT=0" >> ~/.bashrc
```

## 4) Try it out
Ensure XLaunch is running.

In a WSL2 Terminal, run the following lines (separately):
```
sudo apt-get install x11-apps
```

```
xeyes
```

This should spawn two eyes which follow your cursor! Continue to [SSH Setup](#ssh-setup).
If you have issues, reach out to Maxx (jessemaxxwilson@utexas.edu, Discord: Maxx#3164).

## 5) SSH Setup 
If you haven't used Github on your computer before, you will need to add SSH keys. These let github recognize your computer (and this replaced using passwords last year).

### 5.1) Generate SSH Keys
In a WSL2 Terminal:
```
ssh-keygen -t ed25519 -C "PUT_YOUR_EMAIL_HERE@DONT_JUST_COPY_PASTE_THIS.com"
```
Spam "Enter" for all the following questions.

### 5.2) Print Out SSH Key
```
cat ~/.ssh/id_ed25519.pub
```
Copy this to your clipboard. Then, go to github.com and follow these screenshots.

![image](https://github.com/VEXU-GHOST/VEXU_GHOST/assets/47650195/d4107d08-13ee-4a29-ba03-7d72ea4bf5e5)

![image](https://github.com/VEXU-GHOST/VEXU_GHOST/assets/47650195/8cf1f9dc-d258-44c2-b41e-e9426eb5d103)

![image](https://github.com/VEXU-GHOST/VEXU_GHOST/assets/47650195/368667cd-da9a-4e93-b409-2d5823e26cfd)

![image](https://github.com/VEXU-GHOST/VEXU_GHOST/assets/47650195/52daac1c-d046-402d-9dd9-7d2bf45ec4eb)

Paste the key into the highlighted area, name it something like "My Laptop - WSL2" for the Title, and add the key.

## Conclusions
Now you should be all setup with WSL2! Continue Onboarding I [here](https://github.com/VEXU-GHOST/VEXU_GHOST?tab=readme-ov-file#installation).
