# GPS Localization Sim

## By: Ben Grudzien

This project demonstrates an autonomous vehicle localization algorithm. This algorithm utilizes the latitude and longitude of 8 GPS waypoints. I converted those locations to UTM coordinates. Then, I localized the autonomous Audi R8 based on a reference UTM coordinate, the location of the waypoint, and the location of the car. With these locations I was able to create vectors and find the angle of the car relative to the next waypoint using the dot product formula between the vectors. 

![](/Audiobot GPS Algorithm/audi_bot_gps_sim_2)
