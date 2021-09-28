import csv

import krpc

print('Connecting...')
client = krpc.connect()
vessel = client.space_center.active_vessel
kerbin = client.space_center.bodies['Kerbin']

result = [['Altitude', 'Pressure', 'Density', 'Temperature']]

print('Recording...')
for altitude in range(0, 70000, 100):
    result.append([altitude, kerbin.pressure_at(altitude), kerbin.density_at(altitude),
                   kerbin.temperature_at((600000 + altitude, 0, 0), kerbin.reference_frame)])

print('Writing file "density.csv"')
with open('density.csv', 'w', newline='') as file:
    writer = csv.writer(file)
    writer.writerows(result)

print('Finished!')
