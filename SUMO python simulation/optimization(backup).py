import numpy as np
from scipy.optimize import minimize
from scipy.optimize import Bounds
from scipy.optimize import LinearConstraint
from scipy.optimize import NonlinearConstraint

import traci  # noqa

class PartialOptim:
    
    def __init__(self, group, **kwargs):
        #シミュレーションに関する設定
        self.step = kwargs['step']
        self.count = kwargs['count']
        
        #車に関する設定
        self.minGap = kwargs['minGap']
        self.Vdes = kwargs['Vdes']
        self.Vmax = kwargs['Vmax']
        self.Vmin = kwargs['Vmin']
        self.Amax = kwargs['Amax']
        self.Amin = kwargs['Amin']

        
        #最適化に関する設定
        self.dt = kwargs['dt']
        T = kwargs['T']
        self.T = T
        self.N = kwargs['N']
        self.alpha = kwargs['alpha']
        self.w1 = kwargs['w1']
        self.w2 = kwargs['w2']
        self.w3 = kwargs['w3']
        self.w4 = kwargs['w4']
        
        #シミュレーション内の車情報
        self.group = group
        
        p_vehicles = []
        q_vehicles = []
        Lc_p = np.array([])
        Lc_q = np.array([])
        
        for vehicle in group:
            
            #右車線か左車線の振り分け
            route = traci.vehicle.getRoute(vehicle)
            
            if route == ("beg1","middle","end1"):
                p_vehicles.append(vehicle)
                Lc_p = np.append(Lc_p, 0)
       
            if route == ("beg1","middle","end0"):
                p_vehicles.append(vehicle)
                Lc_p = np.append(Lc_p, 1)
    
            if route == ("beg0","middle","end1"):
                q_vehicles.append(vehicle)
                Lc_q = np.append(Lc_q, 1)
    
            if route == ("beg0","middle","end0"):
                q_vehicles.append(vehicle)
                Lc_q = np.append(Lc_q, 0)

        
        p_cars = len(p_vehicles)
        self.p_cars = p_cars
        q_cars = len(q_vehicles)
        self.q_cars = q_cars
        
        cars = p_cars + q_cars
        self.cars = cars
        
        initial_position = np.array([])
        initial_velocity = np.array([])
        
        for vehicle in group:
            initial_position = np.append(initial_position, traci.vehicle.getDistance(vehicle))
            initial_velocity = np.append(initial_velocity, traci.vehicle.getSpeed(vehicle))
            
        self.initial_position = initial_position
        self.initial_velocity = initial_velocity
        self.Lc_p = Lc_p
        self.Lc_q = Lc_q
        Lc = np.append(Lc_p, Lc_q)
        self.Lc = Lc
        
        self.p_end_index = p_cars-1
        self.q_end_index = cars -1
        
        #制約条件
        A = np.zeros((cars*T, 2*cars*(T+1)))
        Beq = np.zeros((cars*T, 2*cars*(T+1)))
        
        eye0 = np.eye(T, T+1, k=0)
        self.eye0 = eye0
        eye1 = np.eye(T, T+1, k=1)
        self.eye1 = eye1
        
        upper_bounds = np.zeros(2*cars*(T+1))
        lower_bounds = np.zeros(2*cars*(T+1))
        
        for i in range(cars):
            index_00 = T*i
            index_01 = T*(i+1)
            index_10 = (T+1)*i
            index_11 = (T+1)*(i+1)
            index_20 = (T+1)*(i+cars)
            index_21 = (T+1)*(i+cars+1)

            #運動力学的制約
            Beq[index_00:index_01, index_10:index_11] = eye0 - eye1
            Beq[index_00:index_01, index_20:index_21] = self.dt*eye1
            #速度制約
            A[index_00:index_01, index_20:index_21] = -eye0 + eye1
            
        
        self.A = A
        self.Beq = Beq

        #境界条件と初期条件、前の最適化の条件
        upper_bounds = np.zeros(2*cars*(T+1))
        lower_bounds = np.zeros(2*cars*(T+1))

        upper_bounds[0:cars*(T+1)] = np.Inf
        lower_bounds[cars*(T+1):] = self.Vmin
        upper_bounds[cars*(T+1):] = self.Vmax

        for n in range(cars):
            upper_bounds[n*(T+1)] = initial_position[n]
            upper_bounds[(cars+n)*(T+1)] = initial_velocity[n]
            lower_bounds[n*(T+1)] = initial_position[n] 
            lower_bounds[(cars+n)*(T+1)] = initial_velocity[n]
        
        self.upper_bounds = upper_bounds
        self.lower_bounds = lower_bounds
        
        self.x0 = self.Cffun_x0()
        
    #x0を生成する関数
    def Cffun_x0(self):
        position = np.zeros((self.cars, self.T+1))
        velocity = np.zeros((self.cars, self.T+1))
        position[:, 0] = self.initial_position 
        velocity[:, 0] = self.initial_velocity
        
        for i in range(self.T):
            for n in range(self.cars):
                if n == 0 or n == self.p_cars:
                    velocity[n, i+1] = np.amin([self.Vmax, velocity[n, i] + self.Amax*self.dt])
                else:
                    velocity[n, i+1] = -0.01 + traci.vehicle.getFollowSpeed(
                        vehID = self.group[n],
                        speed = velocity[n, i],
                        gap = position[n-1, i] - position[n, i] - self.minGap,
                        leaderSpeed = velocity[n-1, i],
                        leaderMaxDecel = -self.Amin
                        )
            position[:, i+1] = position[:, i] + velocity[:, i+1]*self.dt
                
        Fn = np.append(position.ravel(), velocity.ravel())
        return Fn
    
    #車追従モデルによる安全性の制約
    def Cffun(self, x):
        position = x[0:self.cars*(self.T+1)].reshape(-1, self.T+1)
        velocity = x[self.cars*(self.T+1):].reshape(-1, self.T+1)
    
        Fn = np.array([])
        for n in range(self.cars):
            if n == 0 or n == self.p_cars:
                continue
            for i in range(self.T):
                max_next_follower_speed = traci.vehicle.getFollowSpeed(
                    vehID = self.group[n],
                    speed = velocity[n, i],
                    gap = position[n-1, i] - position[n, i] - self.minGap,
                    leaderSpeed = velocity[n-1, i],
                    leaderMaxDecel = -self.Amin
                    )
                Fn = np.append(Fn, max_next_follower_speed -(velocity[n, i]+self.Amin*self.dt))
        return Fn
    
    #目的関数を求める
    def Objfn(self, x):
        cars = self.cars
        p_cars = self.p_cars
        T = self.T
        
        #速度評価
        speed = x[cars*(T+1):]
        speed_eva = np.sum((speed-self.Vdes)**2)
        
        #加速度評価
        accel = (-self.eye0 + self.eye1) @ speed.reshape(-1, T+1).T
        accel_eva = np.sum(accel**2/self.dt**2)
        
        #車線変更リスク評価
        position = x[0:cars*(T+1)].reshape(-1, T+1)
        pep = position[0:p_cars, -1]
        qep = position[p_cars:cars, -1]

        gap = pep.reshape(-1, 1) - qep.reshape(1, -1)

        exp = np.exp(-0.0001*gap**2)
        risk_eva = np.sum(self.Lc_p @ exp) + np.sum(exp @ self.Lc_q)
        #部分最適化した1ブロックの車両同士が離れすぎないように
        block_eva = np.sum(np.where(gap**2 > 100, gap**2, 0))

        Fn = self.w1*speed_eva + self.w2*accel_eva + self.w3*risk_eva + self.w4/T*block_eva
        
        return Fn
        
    #ヤコビアン（勾配）を求める
    def Jacobian(self, x):
        cars = self.cars
        p_cars = self.p_cars
        T = self.T
        dt = self.dt
        Vdes = self.Vdes
        alpha = self.alpha
        w1 = self.w1
        w2 = self.w2
        w3 = self.w3
        w4 = self.w4
        
        jacobian = np.zeros(2*cars*(T+1))
        
        position = x[0:cars*(T+1)].reshape(-1, T+1)
        pep = position[0:p_cars, -1]
        qep = position[p_cars:cars, -1]
        
        gap = pep.reshape(-1, 1) - qep.reshape(1, -1)
        
        grad_exp = np.exp(-alpha*gap**2)*(-2*alpha*gap)
        grad_exp = self.Lc_p.reshape(-1, 1)*grad_exp + self.Lc_q*grad_exp
        
        gap2 = np.where(gap**2 > 100, 2*gap, 0)
    
    
        for i in range(cars):
            #位置
            if i < p_cars:
                jacobian[i*(T+1)+T] = w3*np.sum(grad_exp[i, :]) + w4/T*np.sum(gap2[i, :])
            else:
                jacobian[i*(T+1)+T] = - w3*np.sum(grad_exp[:, i-p_cars]) - w4/T*np.sum(gap2[:, i-p_cars])
            
            #速度
            I = (i+cars)*(T+1)
            jacobian[I] = 2*w1*(x[I] - Vdes) - 2*w2*(x[I+1] - x[I])/dt**2
            jacobian[I+1:I+T] = 2*w1*(x[I+1:I+T] - Vdes) + 2*w2*(x[I+1:I+T] - x[I:I+T-1])/dt**2 - 2*w2*(x[I+2:I+T+1] - x[I+1:I+T])/dt**2
            jacobian[I+T] = 2*w1*(x[I+T] - Vdes) + 2*w2*(x[I+T] - x[I+T-1])/dt**2
            
        return jacobian
        
    def partial_optimize(self):
        
        #制約条件のリスト
        model_constraints = LinearConstraint(self.Beq, -0.001, 0.001)
        acceleration_constraints = LinearConstraint(self.A, self.Amin*self.dt-0.001, self.Amax*self.dt+0.001) 
        carfollowing_constraints = NonlinearConstraint(self.Cffun, 0, self.Vmax)
        
        cons = [model_constraints, acceleration_constraints, carfollowing_constraints]
        
        bound = Bounds(self.lower_bounds-0.001, self.upper_bounds+0.001)
        
        RESULT = minimize(self.Objfn, self.x0, method='SLSQP', jac=self.Jacobian, constraints=cons, bounds=bound)
        result = np.copy(RESULT.x)
        result = result.reshape(-1, self.T+1)
        vehicle_position = result[0:self.cars, 1:]
        vehicle_velocity = result[self.cars:2*self.cars, 1:]
        print(f'optimize={RESULT.success}')
        print(RESULT.message)
        
        return vehicle_position, vehicle_velocity


class TrafficOptim:
    
    def __init__(self, simulation_vehicles, **kwargs):
        self.traffic_settings = kwargs
        self.simulation_vehicles = simulation_vehicles
        
        
    def optimize_vehicles(self):
          
        for iteration, group in enumerate(self.simulation_vehicles):
            to = PartialOptim(group, **self.traffic_settings)
            cars = to.cars
            p_cars = to.p_cars
            T = to.T
            
            if iteration > 0:
                #二回目以降の最適化で、制約条件の被りを防ぐ
                Ceq_delete = np.append(np.arange(0,T), np.arange(p_cars*T, (p_cars+1)*T))
                to.Beq = np.delete(to.Beq, Ceq_delete, 0)
                to.A = np.delete(to.A, Ceq_delete, 0)
                
                #前の部分最適化の値を代入する
                #前回の最適化での最後尾の車両を今回の先頭車とする制約

                to.upper_bounds[1:T+1] = self.previous_p_end_position
                to.upper_bounds[p_cars*(T+1)+1:(p_cars+1)*(T+1)] = self.previous_q_end_position
                to.upper_bounds[cars*(T+1)+1:(cars+1)*(T+1)] = self.previous_p_end_velocity
                to.upper_bounds[(cars+p_cars)*(T+1)+1:(cars+p_cars+1)*(T+1)] = self.previous_q_end_velocity
    
                to.lower_bounds[1:T+1] = self.previous_p_end_position
                to.lower_bounds[p_cars*(T+1)+1:(p_cars+1)*(T+1)] = self.previous_q_end_position
                to.lower_bounds[cars*(T+1)+1:(cars+1)*(T+1)] = self.previous_p_end_velocity
                to.lower_bounds[(cars+p_cars)*(T+1)+1:(cars+p_cars+1)*(T+1)] = self.previous_q_end_velocity
    
                #ついでに初期値x0も変更する
                to.x0[1:T+1] = self.previous_p_end_position
                to.x0[p_cars*(T+1)+1:(p_cars+1)*(T+1)] = self.previous_q_end_position
                to.x0[cars*(T+1)+1:(cars+1)*(T+1)] = self.previous_p_end_velocity
                to.x0[(cars+p_cars)*(T+1)+1:(cars+p_cars+1)*(T+1)] = self.previous_q_end_velocity
            
            vehicle_position, vehicle_velocity = to.partial_optimize()
            
            self.previous_p_end_position = vehicle_position[to.p_end_index, :]
            self.previous_p_end_velocity = vehicle_velocity[to.p_end_index, :]
            self.previous_q_end_position = vehicle_position[to.q_end_index, :]
            self.previous_q_end_velocity = vehicle_velocity[to.q_end_index, :]
            
            p_lanechange_direction = np.where(to.Lc_p == 1, -1, to.Lc_p)
            q_lanechange_direction = to.Lc_q
        
            vehicle_lanechange = np.append(p_lanechange_direction, q_lanechange_direction)
            
            
            
        return vehicle_position, vehicle_velocity, vehicle_lanechange
    
    
        
        
        
        