import numpy as np
from scipy.optimize import minimize
from scipy.optimize import Bounds
from scipy.optimize import LinearConstraint
from scipy.optimize import NonlinearConstraint

import traci  # noqa

class PartialOptimizer:
    
    def __init__(self, leader_position, leader_velocity, leader_lanechange, group, **kwargs):
        #シミュレーションに関する設定
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
        self.alpha = kwargs['alpha']
        self.w1 = kwargs['w1'] #速度重み付け
        self.w2 = kwargs['w2'] #加速度重み付け
        self.w3 = kwargs['w3'] #車線変更リスク重み付け
        
        #左車線と右車線の先行車それぞれ一台ずつの位置と速度
        self.leader_right_position = leader_position[0]
        self.leader_right_velocity = leader_velocity[0]
        self.leader_left_position = leader_position[1]
        self.leader_left_velocity = leader_velocity[1]
        
        #groupは前からの位置でソート済み
        #0車線の車をright_vehicles, 1車線の車をleft_vehiclesとする
        routes = [traci.vehicle.getLaneIndex(vehicle) for vehicle in group]
        self.right_vehicles = [v for i, v in enumerate(group) if routes[i] == 0]
        self.left_vehicles = [v for i, v in enumerate(group) if routes[i] == 1]
        self.group = self.right_vehicles + self.left_vehicles
        
        self.initial_position = np.array([traci.vehicle.getDistance(v) for v in self.group])
        self.initial_velocity = np.array([traci.vehicle.getSpeed(v) for v in self.group])

        right_vehicles_will_change_lanes = []
        left_vehicles_will_change_lanes = []
        for vehicle in self.right_vehicles:
            if int(vehicle[5:]) % 4 == 0:
                right_vehicles_will_change_lanes.append(True)
            else:
                right_vehicles_will_change_lanes.append(False)
        
        for vehicle in self.left_vehicles:
            if int(vehicle[5:]) % 4 == 2:
                left_vehicles_will_change_lanes.append(True)
            else:
                left_vehicles_will_change_lanes.append(False)
        
        self.right_leader_will_change_lanes = leader_lanechange[0]
        self.left_leader_will_change_lanes = leader_lanechange[1]
        self.right_vehicles_will_change_lanes = right_vehicles_will_change_lanes
        self.left_vehicles_will_change_lanes = left_vehicles_will_change_lanes
        self.will_change_lanes = self.right_vehicles_will_change_lanes + self.left_vehicles_will_change_lanes
        
        self.right_vehicle_count = len(self.right_vehicles)
        self.left_vehicle_count = len(self.left_vehicles)
        self.vehicle_count = len(self.group)
        
        #制約条件
        T = self.T
        vehicle_count = self.vehicle_count
        
        self.eyeN = np.eye(self.N)
        self.eye0 = np.eye(T, T+1, k=0)
        self.eye1 = np.eye(T, T+1, k=1)
        self.A = np.kron(self.eyeN, -self.eye0 + self.eye1) + 0
        self.lb = self.Amin * self.dt
        self.ub = self.Amax * self.dt
        
        #境界条件と初期条件、前の最適化の条件
        self.lower_bounds = self.Vmin * np.ones(vehicle_count*(T+1))
        self.upper_bounds = self.Vmax * np.ones(vehicle_count*(T+1))
        self.lower_bounds[0:vehicle_count*(T+1):T+1] = self.initial_velocity - 0.001
        self.upper_bounds[0:vehicle_count*(T+1):T+1] = self.initial_velocity + 0.001
        
        self.x0 = self.generate_x0()
        
        #下半分が0、対角成分が1、上半分が1、0行目が0の行列を作る
        self.nearly_eye = np.ones((T+1, T+1))
        for i in range(T+1):
            self.nearly_eye[i, 0:i] = 0
            
        self.nearly_eye[0, :] = 0
        
        #object_functionで用いる
        right_index = self.right_leader_will_change_lanes + self.right_vehicles_will_change_lanes
        left_index = self.left_leader_will_change_lanes + self.left_vehicles_will_change_lanes
        self.risk_matrix = np.array(right_index).reshape(-1, 1, 1) + np.array(left_index).reshape(1, -1, 1)
        if self.right_leader_will_change_lanes == True and self.left_leader_will_change_lanes == True:
            self.risk_matrix[0, 0, :] = 0 
        

    def generate_x0(self):
        #x0を生成する関数
        T = self.T
        position = np.zeros((self.vehicle_count, T+1))
        velocity = np.zeros((self.vehicle_count, T+1))
        position[:, 0] = self.initial_position 
        velocity[:, 0] = self.initial_velocity
        
        for n in range(self.vehicle_count):
            if self.right_vehicle_count != 0 and n == 0:
                leader_position = self.leader_right_position
                leader_velocity = self.leader_right_velocity
            elif n == self.right_vehicle_count:
                leader_position = self.leader_left_position
                leader_velocity = self.leader_left_velocity
            else:
                leader_position = position[n-1, :]
                leader_velocity = velocity[n-1, :]
            
            if leader_position.size > 0:
                for t in range(T):
                    velocity[n, t+1] = traci.vehicle.getFollowSpeed(
                        vehID = self.group[n],
                        speed = velocity[n, t],
                        gap = leader_position[t] - position[n, t] - self.minGap,
                        leaderSpeed = leader_velocity[t],
                        leaderMaxDecel = -self.Amin
                        )
                    position[n, t+1] = position[n, t] + velocity[n, t+1]*self.dt
            else:
                for t in range(T):
                    velocity[n, t+1] = min([velocity[n, t] + self.Amax*self.dt, self.Vmax])
                    position[n, t+1] = position[n, t] + velocity[n, t+1]*self.dt
            
        return velocity.reshape(-1)
    
    #速度から車の位置を計算する
    def calculate_position(self, x):
        #xは一次元配列を想定 型はnp.ndarray
        velocity = self.dt * x.reshape(-1, self.T+1) 
        position = self.initial_position.reshape(-1, 1) + velocity @ self.nearly_eye    
        return position.reshape(-1)
    
    #車追従モデルによる安全性の制約
    def car_following_model(self, x):
        T = self.T
        position = self.calculate_position(x)
        position = position.reshape(-1, T+1)
        velocity = x.reshape(-1, T+1)
    
        Fn = np.ones(self.vehicle_count * T)
        for n in range(self.vehicle_count):
            if self.right_vehicle_count != 0 and n == 0:
                leader_position = self.leader_right_position
                leader_velocity = self.leader_right_velocity
            elif n == self.right_vehicle_count:
                leader_position = self.leader_left_position
                leader_velocity = self.leader_left_velocity
            else:
                leader_position = position[n-1, :]
                leader_velocity = velocity[n-1, :]
            
            if leader_position.size > 0:
                for t in range(T):
                    max_next_follower_speed = traci.vehicle.getFollowSpeed(
                        vehID = self.group[n],
                        speed = velocity[n, t],
                        gap = leader_position[t] - position[n, t] - self.minGap,
                        leaderSpeed = leader_velocity[t],
                        leaderMaxDecel = -self.Amin
                        )
                    Fn[n*T + t] = max_next_follower_speed - (velocity[n, t]+self.Amin*self.dt)
                
        return Fn
    
    #目的関数を求める
    def objective_function(self, x):
        vehicle_count = self.vehicle_count
        right_vehicle_count = self.right_vehicle_count
        T = self.T
        
        #速度評価
        speed = x.reshape(-1, T+1)[:, 1:]
        speed_evaluation = np.sum((speed - self.Vmax)**2)
        
        #加速度評価
        accel = (-self.eye0 + self.eye1) @ x.reshape(-1, T+1).T
        accel_evaluation = np.sum(accel[1:, :]**2/self.dt**2)
        
        #車線変更リスク評価
        position = self.calculate_position(x)
        right_position = np.append(self.leader_right_position, position[0:right_vehicle_count*(T+1)])
        left_position = np.append(self.leader_left_position, position[right_vehicle_count*(T+1):vehicle_count*(T+1)])
        distance = right_position.reshape(-1, 1, T+1) - left_position.reshape(1, -1, T+1)
        distance = distance[:, :, 1:]
        distance = self.risk_matrix * distance
        exp = np.exp(-self.alpha*distance**2)
        exp = self.risk_matrix * exp
        risk_evaluation = np.sum(exp)
        
        Fn = self.w1*speed_evaluation + self.w2*accel_evaluation + self.w3*risk_evaluation
        
        return Fn
        
    def partial_optimize(self):
        #1グループに対して、最適化を行う
        #制約条件のリスト
        acceleration_constraints = LinearConstraint(self.A, self.lb, self.ub) 
        carfollowing_constraints = NonlinearConstraint(self.car_following_model, 0, self.Vmax)
        
        #最適化を行う
        RESULT = minimize(self.objective_function,
                          x0 = self.x0,
                          method = 'SLSQP',
                          constraints = [acceleration_constraints, carfollowing_constraints],
                          bounds = Bounds(self.lower_bounds, self.upper_bounds),
                          options = {"disp":False,"maxiter":10}
                          )
        """
        if RESULT.success == False:
            print(f'optimize={RESULT.success}')
            print(RESULT.message)
            print("x=", RESULT.x)
            print(self.A @ RESULT.x)
            print("car_following_model=", self.car_following_model(RESULT.x))
        """

        result = RESULT.x
        result = result.reshape(-1, self.T+1)
        
        group_place = self.calculate_position(result)
        group_place = group_place.reshape(-1, self.T+1)
        group_speed = result
        
        return group_place, group_speed
        
        
        
        