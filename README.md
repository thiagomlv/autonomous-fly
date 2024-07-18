# Harpia's first autonomous fly

Here is described how to perform the first autonomous fly.

# Setting the environment

1. Run the code bellow. This will install all the required Linux system packages and python modules nedeed.

```shell
curl -s -L https://raw.githubusercontent.com/thiagomlv/autonomous-fly/main/environment-setup/environment_setup.sh | /usr/bin/bash
```

2. To confirm your environment is done, verify if the directory _home/ardu-sim/_ exits and contains the files _arducopter_, _ardu-sim.sh_ and the directory _parameters_.  

3. If the directory wasn't created at the specified path os any files/directory are missing, try to run the code one more time, and check it again.

4. If it is all right,run:

```shell
cd home/ardu-sim/
./ardu-sim.sh
screen -ls
```

5. As output, we should see two screens: the _proxy_ and the _vehicle_. This confirms that the environment is done and all the requiremets were installed/created.

# Conecting to the drone (Simulation)

1. Run the code:

```shell
mavproxy.py --master=127.0.0.1:14550
```

2. To see the mini-map, run:

```shell
module load map
```