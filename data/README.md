## Data Folder

Place your CSV files here!

Right now its expected the CSVs will have the following columns in this order:
`time,accelx,accely,accelz,gyrox,gyroy,gyroz,magx,magy,magz,altitude,pressure,temp`

### Example CSV
```
time,accelx,accely,accelz,gyrox,gyroy,gyroz,magx,magy,magz,altitude,pressure,temp
3674269.348155,9.84407,0.82313,0.0,-0.00244346,-0.00244346,0.00641408,-55.5539,-15.1564,17.2464,2.32248,1021.35,30.2456
3674350.158699,9.84407,0.842273,-0.100498,-0.00702495,-0.0175624,0.00366519,-55.6416,-15.1272,17.0418,2.66052,1021.37,30.2545
3674430.969242,9.84407,0.89013,-0.330209,-0.00962113,-0.0138972,0.00397062,-55.8755,-15.2002,17.9772,2.82953,1021.38,30.2545
```

### Units
- `time` is in miliseconds
- `accelx`, `accely`, `accelz` are in m/s^2
- `gyrox`, `gyroy`, `gyroz` are in rad/s
- `magx`, `magy`, `magz` are in uT
- `altitude` is in meters
- `pressure` is in hPa
- `temp` is in degrees Celsius