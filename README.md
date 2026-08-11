# UAS Pi 5 Autonomous Platforms

| Project | Description |
|---|---|
| [CrowDrone](./CrowDrone/) | Aerial platform |
| [GroundRobots](./GroundRobots/turtlebot-navigation/) | TurtleBot2 indoor autonomous navigation |

## Devices

| Platform | Hostname | Username | SSH |
|---|---|---|---|
| Drone | `bolzpi` | `pi` | `ssh pi@bolzpi.local` |
| Ground robot | `bolzpi2` | `uas` | `ssh uas@bolzpi2.local` |

Both run headless over an iPhone hotspot (`172.20.10.x`). If `.local` mDNS fails,
find the IP with `arp -a | grep 172.20.10` and SSH to that instead.

Shared setup: [Pi Hotspot & SSH Guide](./Pi_Hotspot_SSH_Guide.md)
