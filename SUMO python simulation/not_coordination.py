import numpy as np
import matplotlib.pyplot as plt


from sumolib import checkBinary  # noqa
import traci  # noqa

sumoBinary = checkBinary('sumo-gui')
sumoCmd = [sumoBinary, "-c", "yy.sumocfg"]

traci.start(sumoCmd) #traciの開始

fuel_consumption = np.array([])
traffic_volume = np.array([])
vehicle_count = 0
step = 0

while step < 3600:
    
    #SUMOのシミュレーションを進める
    traci.simulationStep()

    #挿入された車両の車線変更を禁止する
    traci_inserted_vehicles = list(traci.simulation.getLoadedIDList())
    for n in traci_inserted_vehicles:

        if vehicle_count % 5 == 0:
            #車線変更する
            traci.vehicle.setColor(n,(0,100,0,255))
            route = traci.vehicle.getRoute(n)

            if route == ("beg1","middle","end1"):
                traci.vehicle.changeTarget(
                    vehID = n,
                    edgeID = "end0"
                    )

            if route == ("beg0","middle","end0"):
                traci.vehicle.changeTarget(
                    vehID = n,
                    edgeID = "end1"
                    )
        
        else:
            #車線変更しない
            traci.vehicle.setColor(n,(255,255,255,255))

        vehicle_count += 1

    #測定
    vehicle_list = traci.vehicle.getIDList()
    #車一台あたりの平均燃料消費量の測定
    total_fuel_consumption_per_sec = np.array([])
    for n in vehicle_list:
        total_fuel_consumption_per_sec = np.append(total_fuel_consumption_per_sec,traci.vehicle.getFuelConsumption(n))
    fuel_consumption = np.append(fuel_consumption, total_fuel_consumption_per_sec)
    
    #交通量の測定
    vehicle_number = traci.simulation.getLoadedNumber()
    traffic_volume = np.append(traffic_volume, vehicle_number)
    #一秒経過
    step += 1

traci.close()

total_fuel_consumption = np.sum(fuel_consumption)
traffic_volume = np.sum(traffic_volume)
left = np.array([1, 2])
height = np.array([total_fuel_consumption, traffic_volume])
plt.bar(left, height, tick_label = ["fuel_consumption", "traffic_volume"])

print(f"fuel_consumption={total_fuel_consumption}")
print(f"traffic_volume={traffic_volume}")