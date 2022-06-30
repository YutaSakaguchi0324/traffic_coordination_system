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

from optimization import PartialOptimizer


#各定数の設定
traffic_settings = {
    #シミュレーションに関する設定
    'simulation_time': 600, # [s] シミュレーションを行う時間（秒）
    'dt': 0.5, # [s] 微小時間（1ステップ時間）

    #車に関する設定
    'minGap': 7, # [m] 最小車間距離
    'Vmax': 23, # [m/s] 最高速度
    'Vmin': 0, # [m/s] 最低速度
    'Amax': 0.73, # [m/s^2] 最大加速度
    'Amin': -1.67, #[m/s^2] 最小加速度

    #最適化に関する設定
    'T': 6, # [-] 最適化する時間ステップ区間 ホライゾン
    'N': 2, # [-] １つの部分最適化の車の数
    'starting_line_position': 200, # [m]　最適化を行う道路の範囲　スタートライン
    'goal_line_position': 800, # [m] 最適化を行う道路の範囲　ゴールライン
    'alpha': 0.0005, # [-] 衝突リスク定数
    'w1': 0.01, # [-] 重み付け
    'w2': 0.1, # [-] 重み付け
    'w3': 2, # [-] 重み付け
    }

class TrafficCoordinationSystem:
    
    def __init__(self, **kwargs):
        #シミュレーションに関する設定
        self.simulation_time = kwargs['simulation_time']
        self.dt = kwargs['dt']

        #車に関する設定
        self.minGap = kwargs['minGap']
        self.Vmax = kwargs['Vmax']
        self.Vmin = kwargs['Vmin']
        self.Amax = kwargs['Amax']
        self.Amin = kwargs['Amin']

        #最適化に関する設定
        self.T = kwargs['T']
        self.N = kwargs['N']
        self.starting_line_position = kwargs['starting_line_position']
        self.goal_line_position = kwargs['goal_line_position']

    def start(self):
        #SUMOの起動
        sumoBinary = checkBinary('sumo-gui')
        sumoCmd = [sumoBinary, "-c", "two-lane_highway.sumocfg"]
        #traciの開始
        traci.start(sumoCmd)

    def change_mode(self, inserted_vehicles):
        #挿入された車両の車線変更を禁止する
        for vehicle in inserted_vehicles:
            traci.vehicle.setSpeedMode(
                vehID = vehicle,
                sm = 32
                )
            traci.vehicle.setLaneChangeMode(
                vehID = vehicle,
                lcm = 512
                )
            
            if traci.vehicle.getLaneIndex(vehicle) == 0:
                if int(vehicle[5:]) % 4 == 0:
                    #車線変更の予定がある
                    traci.vehicle.setColor(vehicle, (0, 100, 0, 255))
            else:
                if int(vehicle[5:]) % 4 == 2:
                    traci.vehicle.setColor(vehicle, (0, 100, 0, 255))
                        

    def delete_group_from(self, simulation_vehicles):
        #simulation_vehicle車列から、範囲外のグループを削除する
        new_simulation_vehicles = []
        for group in simulation_vehicles:
            if traci.vehicle.getDistance(group[0]) <= self.goal_line_position:
                new_simulation_vehicles.append(group)
            else:
                for vehicle in group:
                    traci.vehicle.remove(vehicle)
        
        return new_simulation_vehicles

    def sort_cars(self, vehicles):
        #vehiclesの車を前から順に並べ替える
        position = [traci.vehicle.getDistance(vehicle) for vehicle in vehicles]
        index = np.argsort(position)[::-1]
        vehicles = [vehicles[i] for i in index]
        
        return vehicles
    
    def divide_right_left(self, vehicles):
        right_vehicles = [v for v in vehicles if traci.vehicle.getLaneIndex(v) == 0]
        left_vehicles = [v for v in vehicles if traci.vehicle.getLaneIndex(v) == 1]
        vehicles = right_vehicles + left_vehicles
        
        return vehicles

    def select_vehicles(self, simulation_vehicles, waiting_vehicles):
        #待機車列からシミュレーションに入れる車を選定する
        simulation_vehicles = [vehicle for group in simulation_vehicles for vehicle in group]
        vehicles = simulation_vehicles + waiting_vehicles
        vehicles = self.sort_cars(vehicles)
        quotient = len(vehicles) // self.N
        remainder = len(vehicles) % self.N
        simulation_vehicles = [vehicles[i*self.N:(i+1)*self.N] for i in range(quotient)]   
        simulation_vehicles = [self.divide_right_left(group) for group in simulation_vehicles ]
        waiting_vehicles = vehicles[self.N*quotient:self.N*quotient + remainder]
        
        return simulation_vehicles, waiting_vehicles

    def optimize_vehicles(self, simulation_vehicles):
        #車のT秒後までの運転計画を最適化する
        group_count = len(simulation_vehicles)
        #simulation_vehicles全体の記録
        every_vehicle_velocity = np.zeros((self.N * group_count, self.T))
        
        leader_right_position = np.array([])
        leader_right_velocity = np.array([])
        leader_right_lanechange = []
        leader_left_position = np.array([])
        leader_left_velocity = np.array([])
        leader_left_lanechange = []

        for iteration, group in enumerate(simulation_vehicles):
            leader_position = [leader_right_position, leader_left_position]
            leader_velocity = [leader_right_velocity, leader_left_velocity]
            leader_lanechange = [leader_right_lanechange, leader_left_lanechange]
            po = PartialOptimizer(leader_position, leader_velocity, leader_lanechange, group, **traffic_settings)
            right_vehicle_count = po.right_vehicle_count
                
            #返り値のgroup_place, group_speedは2次元配列が望ましい
            group_place, group_speed = po.partial_optimize()
            group_will_change_lanes = po.will_change_lanes
            
            #速度と車線変更ブール値の記録
            every_vehicle_velocity[iteration*self.N:(iteration+1)*self.N, :] = group_speed[:, 1:]
            
            if po.right_vehicle_count == 0:
                leader_left_position = group_place[-1, :]
                leader_left_velocity = group_speed[-1, :]
                leader_left_lanechange = [group_will_change_lanes[-1]]
            elif po.left_vehicle_count == 0:
                leader_right_position = group_place[-1, :]
                leader_right_velocity = group_speed[-1, :]
                leader_right_lanechange = [group_will_change_lanes[-1]]
            else:
                leader_right_position = group_place[po.right_vehicle_count-1, :]
                leader_right_velocity = group_speed[po.right_vehicle_count-1, :]
                leader_right_lanechange = [group_will_change_lanes[po.right_vehicle_count-1]]
                leader_left_position = group_place[-1, :]
                leader_left_velocity = group_speed[-1, :]
                leader_left_lanechange = [group_will_change_lanes[-1]]
                
            #returnする値は二次元配列が望ましい
  
        return every_vehicle_velocity

    def instruct_vehicles(self, simulation_vehicles, vehicle_velocity, elapsed_time):
        #計算した値を車に指示する
        if len(simulation_vehicles) > 0:
            for i in range(len(simulation_vehicles)):
                for n in range(self.N):
                    traci.vehicle.setSpeed(simulation_vehicles[i][n], vehicle_velocity[i*self.N + n, elapsed_time])


#メイン実行コード
if __name__ =="__main__":

    tcs = TrafficCoordinationSystem(**traffic_settings)
    #SUMOとTraCIを起動する
    tcs.start()
    simulation_vehicles = []
    waiting_vehicles = []

    for step in range(tcs.simulation_time):
        elapsed_step = step % tcs.T
        #シミュレーションを1ステップ進める
        traci.simulationStep()
        #シミュレーションに新しく挿入された車のリスト
        inserted_vehicles = list(traci.simulation.getLoadedIDList())
        #自動で運転する設定から、すべてpython上で操作できる設定に変更
        tcs.change_mode(inserted_vehicles)
        #挿入された車を待機車列に入れる
        waiting_vehicles = waiting_vehicles + inserted_vehicles
        
        #T秒ごとに最適化する
        if elapsed_step == 0:
            #一定距離を過ぎたグループの削除
            simulation_vehicles = tcs.delete_group_from(simulation_vehicles)
            #シミュレーションに追加する車を選定する
            simulation_vehicles, waiting_vehicles = tcs.select_vehicles(simulation_vehicles, waiting_vehicles)
            #シミュレーションの車の最適化をする
            every_vehicle_velocity = tcs.optimize_vehicles(simulation_vehicles)

        #出力された運転計画に従って車を動かす
        #引数に指示したい車とその軌道を入れる
        #もし運転計画の時間を過ぎたら、sumoの自動運転に切り替えたい
        tcs.instruct_vehicles(simulation_vehicles, every_vehicle_velocity, elapsed_step)
    
    traci.close()
