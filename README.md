
# When you want to run and manage this project, you should follow the steps below:

## 0. You need to be able to establish a serial connection to a piece of hardware that hosts an ardupilot firmware
- If you don't have this setup, none of the following steps matters.

## 1. Activate your virtual environment

- For Linux and macOS:
```
source <path_to_virtualenv>/bin/activate
```

- For Windows:
```
<path_to_virtualenv>\Scripts\activate
```

But note that this project is built and tested on Linux (Ubuntu).
If you want to run this on Windows, you might have to make some changes.


## 2. Install all dependencies

```
pip install -r requirements.txt
```

This command will install the exact versions of the packages 
listed in the requirements.txt file, ensuring compatibility 
and consistency across different environments.


## 3) When you add dependencies to this project, use the following command:

```
pip freeze > requirements.txt
```

This will update the `requirements.txt` file. 


On Linux system, if you want to open a port to connect to the pixhawk 4 or the telemetry hardware,
you need to add your current user ($USER) to the `dialout` group:

```
sudo adduser $USER dialout
```

For the change of your group to take effect, you need to restart the linux operating system.
To check if you are included in the group:

```
> id
groups=4(adm),20(dialout),24(cdrom),27(sudo),30(dip),46(plugdev),122(lpadmin),135(lxd),136(sambashare)
```

As you can see, the current user is in the group `dialout`.