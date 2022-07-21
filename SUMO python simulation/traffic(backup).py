"""
# -*- coding: utf-8 -*-

@author: Yuta Sakaguchi
"""
import os, sys
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")


import numpy as np

from sumolib import checkBinary  # noqa
import traci  # noqa

from optimization(backup) import TrafficOptim


#各定数の設定
traffic_settings = {
    #シミュレーションに関する設定
    'step': 0, # [s] 経過時間
    'count': 0, # [-] 車の数のカウント
    'simulation_time': 600, # [s] シミュレーションを行う時間（秒）
    'max_group_number': 5, # [-] シミュレーションを行う最大のグループの数
    
    #車に関する設定
    'minGap': 20, # [m] 最小車間距離
    'Vdes': 20, # [m/s] 車の理想速度
    'Vmax': 20, # [m/s] 最高速度
    'Vmin': 0, # [m/s] 最低速度
    'Amax': 2.6, # [m/s^2] 最大加速度
    'Amin': -4.5, #[m/s^2] 最小加速度
    
    #最適化に関する設定
    'dt': 1, # [s] 微小時間（1ステップ時間）
    'T': 4, # [-] 最適化する時間区間 ホライゾン
    'N': 4, # [-] １つの部分最適化の車の数
    'alpha': 0.0001, # [-] 衝突リスク係数
    'w1': 0.01, # [-] 重み付け
    'w2': 0.01, # [-] 重み付け
    'w3': 20, # [-] 重み付け
    'w4': 0.003 # [-] 重み付け
    }

class TrafficSim:
    
    def __init__(self, **kwargs):
        
        #シミュレーションに関する設定
        self.step = kwargs['step']
        self.count = kwargs['count']
        self.simulation_time = kwargs['simulation_time']
        self.max_group_number = kwargs['max_group_number']
        
    
        #車に関する設定
        self.minGap = kwargs['minGap']
        self.Vdes = kwargs['Vdes']
        self.Vmax = kwargs['Vmax']
        self.Vmin = kwargs['Vmin']
        self.Amax = kwargs['Amax']
        self.Amin = kwargs['Amin']

        #最適化に関する設定
        self.dt = kwargs['dt']
        self.T = kwargs['T']
        self.N = kwargs['N']
        
        self.elapsed_time = 0
        self.simulation_vehicles = []
        self.next_inserted_vehicles = []
        self.waiting_vehicles = []
        
        #最適化結果の記録する変数
        self.position_memory = np.array([])
        self.velocity_memory = np.array([])
        self.lanechange_memory = np.array([])
        
    def start(self):
        #SUMOの起動
        sumoBinary = checkBinary('sumo-gui')
        sumoCmd = [sumoBinary, "-c", "yy.sumocfg"]
        #traciの開始
        traci.start(sumoCmd) 
        
    def make_inserted_vehicles_controllable(self):
        #挿入された車両の車線変更を禁止する
        traci_inserted_vehicles = list(traci.simulation.getLoadedIDList())
        
        for n in traci_inserted_vehicles:
            traci.vehicle.setLaneChangeMode(
                vehID = n,
                lcm = 512
                )   
    
            if self.count % 5 == 0:
                #車線変更する
                traci.vehicle.setColor(n ,(0, 100, 0, 255))
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
    
            self.count += 1
        
    def add_inserted_vehicles_to_simulation_vehicles(self):
        
        #新しい最適化にむけて、N台になるまで待機する車群
        inserted_vehicles = self.next_inserted_vehicles + list(traci.simulation.getLoadedIDList())
        self.next_inserted_vehicles = inserted_vehicles[self.N - len(self.waiting_vehicles):]
        self.waiting_vehicles.extend(inserted_vehicles[0:self.N - len(self.waiting_vehicles)])
        
        
        if len(self.waiting_vehicles) == self.N:
            p_vehicles = []
            q_vehicles = []
            
            for vehicle in self.waiting_vehicles:
                
                #右車線か左車線の振り分け
                route = traci.vehicle.getRoute(vehicle)
                
                if route == ("beg1","middle","end1"):
                    p_vehicles.append(vehicle)
           
                if route == ("beg1","middle","end0"):
                    p_vehicles.append(vehicle)
        
                if route == ("beg0","middle","end1"):
                    q_vehicles.append(vehicle)
        
                if route == ("beg0","middle","end0"):
                    q_vehicles.append(vehicle)
        
            self.simulation_vehicles.append(p_vehicles + q_vehicles)
            
            #前回の部分最適化の最後尾の車を次の部分最適化で扱うので、次の挿入リストに加える
            self.next_inserted_vehicles[0:0] = [p_vehicles[-1], q_vehicles[-1]]
            
    def change_lane(self):
        #車線変更させる
        lanechange_vehicles = self.simulation_vehicles[0]
        lanechange_direction = self.lanechange_memory[0,:]

        if  np.any(lanechange_direction != 0):
            #リストから除外する車両を車線変更させる
            lanechange_vehicles = [n for i,n in enumerate(lanechange_vehicles) if lanechange_direction[i] != 0]
            lanechange_direction = lanechange_direction[np.where(lanechange_direction != 0)]

            for vehicle, direction in zip(lanechange_vehicles, lanechange_direction):
                lane_index = traci.vehicle.getLaneIndex(vehicle)
                if int(direction) == 1 and lane_index == 0:
                    traci.vehicle.changeLane(
                        vehID = vehicle,
                        laneIndex = 1,
                        duration = 3
                        )
                if int(direction) == -1 and lane_index == 1:
                    traci.vehicle.changeLane(
                        vehID = vehicle,
                        laneIndex = 0,
                        duration = 3
                        )
                        
    def delete(self):
        #車両リストを削除する
        for vehicle in self.simulation_vehicles[0]:
            traci.vehicle.setSpeed(vehicle, -1)
        del self.simulation_vehicles[0]
        self.position_memory = self.position_memory[1:,:,:]
        self.velocity_memory = self.velocity_memory[1:,:,:]
        self.lanechange_memory = self.lanechange_memory[1:,:]
        
    def instruct(self):
        #計算した値を車に指示する
            for i in range(len(self.simulation_vehicles)):
                for n in range(self.N):
                    traci.vehicle.setSpeed(self.simulation_vehicles[i][n], self.velocity_memory[i, n, self.elapsed_time])
            self.elapsed_time += 1
    
    def main(self):
        #SUMOとTraCIを起動する
        self.start()
        
        while self.step < self.simulation_time:
            
            traci.simulationStep()
            self.make_inserted_vehicles_controllable()
            self.add_inserted_vehicles_to_simulation_vehicles()
            
            if len(self.waiting_vehicles) == self.N:
                self.waiting_vehicles = []
                to = TrafficOptim(self.simulation_vehicles, **traffic_settings)
                
                vehicle_position, vehicle_velocity, vehicle_lanechange = to.optimize_vehicles()
                
                self.position_memory = np.append(self.position_memory, vehicle_position).reshape(-1, self.N, self.T)
                self.velocity_memory = np.append(self.velocity_memory, vehicle_velocity).reshape(-1, self.N, self.T)
                self.lanechange_memory = np.append(self.lanechange_memory, vehicle_lanechange).reshape(-1, self.N)

                self.elapsed_time = 0
                
            if len(self.simulation_vehicles) == self.max_group_number - 1:
                self.change_lane()
                
            if len(self.simulation_vehicles) == self.max_group_number:
                self.delete()
            
            if len(self.simulation_vehicles) > 0:
                self.instruct()
                
            self.step += 1
            
#メイン実行コード
if __name__ =="__main__":
    
    ts = TrafficSim(**traffic_settings)
    ts.main()