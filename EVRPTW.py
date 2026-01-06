import gurobipy as gp
from gurobipy import GRB
from Data import A_P, A_RS, A_C, A_Bundle, metro_timetable_updated, N, distances_robots_streets, distances_camions

class modelo:
    def __init__(self, epsilon, battery_capacity):
        self.epsilon = epsilon
        self.battery_capacity = battery_capacity
        self.M = 10000
        self.metro_timetable_updated = metro_timetable_updated
        self.distances_robots_streets = distances_robots_streets
        self.distances_camions = distances_camions
        self.A_P = A_P
        self.A_RS = A_RS
        self.A_C = A_C
        self.A_Bundle = A_Bundle
        self.nodes = list(range(25))
        self.num_nodes = 25
        self.demands = {key: value.get("demand", 0) for key, value in N.items()}   # demands = { 0: 0, 1: 0, 2: 0,3: 0,4: 0,5: 0,6: 0,7: 300, 8: 200,9: 0,10: 0}
        self.service_time = {key: value.get("service_time", 0) for key, value in N.items()}   # service_time = { 0: 0, 1: 0, 2: 0,3: 0,4: 0,5: 0,6: 0,7: 300, 8: 200,9: 0,10: 0}
        self.N_C = [clave for clave, valor in N.items() if valor["type"] == "customer"]   # N_C = [7, 8]
        self.N_F    = [clave for clave, valor in N.items() if valor["type"] == "depot"]   # N_F = [9, 10]
        self.N_T  = [clave for clave, valor in N.items() if valor["type"] == "station"]   # N_T = [1, 2, 3, 4, 5, 6]
        self.N_TP  = [clave for clave, valor in N.items() if valor["type"] == "station_copy"]   # N_TP = [11, 12, 13, 14, 15, 16]
        self.stations_and_copies = self.N_T + self.N_TP
        self.A = A_RS + A_P
        robot_speeds = {"Turbo":  {"speed": 133.33, "coef_BC": 0.4}, "Medium":  {"speed": 100.2, "coef_BC": 0.3}, "Eco": {"speed": 66.6, "coef_BC": 0.2}}
        
        self.K = list(robot_speeds.keys())   # ["Turbo","Medium","Eco"]
        # arcos de calle con modo: (i, j, v)
        self.street_arcs = [(i, j, v) for (i, j) in self.A_RS for v in self.K]
        self.time_street = {}     # (i,j,v):tiempo
        self.battery_consumption = {}  # (i,j,v): batería
        distances_on_street = {(i,j): d for (i,j), d in self.distances_robots_streets.items() if (i,j) in self.A_RS}
        for (i, j, v) in self.street_arcs:
            dist = distances_on_street[(i, j)]
            TT = robot_speeds[v]["speed"]
            BC = robot_speeds[v]["coef_BC"]
            t = dist / TT       
            b = dist * BC
            self.time_street[(i, j, v)]    = t
            self.battery_consumption[(i, j, v)] = b           
        
        veloc_cam = 833.33
        self.true_distances_camions_dict = {(i,j): d for (i,j), d in self.distances_camions.items() if (i,j) in self.A_C}
        self.time_camions = {}
        for (i, j), dist_c in self.true_distances_camions_dict.items():
            tc = dist_c / veloc_cam
            self.time_camions[(i, j)] = tc

        self.arcs_metro_list = []   # arcos del metro: (i, j, t)
        self.arr_time_i = {} # arr_time[(i,j,t)]
        self.dep_time_i = {} # dep_time[(i,j,t)]
        self.arr_time_j = {} # dep_time[(i,j,t)]
        self.line_arc = {}  # line_arc[(i,j,t)]
        self.TTM = {}  # travel_time_metro[(i,j,t)]
        for (i, j), trips in metro_timetable_updated.items():
            for t, dicc in trips.items():   # t = viaje
                metro_arc = (i, j, t)
                self.arcs_metro_list.append(metro_arc)
                self.arr_time_i[metro_arc] = dicc["arr_i"] # Metto al dizionario self.arr_time ogni arco con il suo arr
                self.dep_time_i[metro_arc] = dicc["dep_i"] # Metto al dizionario self.dep_time ogni arco con il suo dep
                self.arr_time_j[metro_arc] = dicc["arr_j"] # Metto al dizionario self.dep_time ogni arco con il suo dep
                self.line_arc[metro_arc] = dicc["line"] # Metto al dizionario self.line_arc ogni arco con il suo line 
                self.TTM[metro_arc] = dicc["travel_time"] # Metto al dizionario self.TTM ogni arco con il suo TTM 
        
        self.init_depot = [0]              # Identifico el depot
        self.NF1 = [9]
        self.NF2 = [10]
        self.init_depot_TW = [589,840]    # Identifico las horas al dia que trabaja el depot inicial
        self.final_depots_TW = [480, 840]                    # Horario de trabajo de los nodos finales      
        self.balance_nodes = [1, 2, 3, 4, 5, 6, 7, 8, 11, 12, 13, 14, 15, 16, 17, 18, 19] # Nodi dove ci vuole bilancio di flusso
        self.V_R = range(5)
        self.robot_capacity = 800
        # 8000, 445    , puesta a 500 escoge medium      con 430 activa 1 truck
        # self.battery_capacity = 8000
        # self.Time_windows_customers = {7: (600, 700), 8: (700, 800)}
        self.Time_windows_customers = {7: (607, 637), 8: (625, 660), 17: (645, 710), 18: (680, 720), 19: (690, 740), 20: (720, 840), 21: (710, 840), 22: (780, 820), 23: (780, 810), 24: (600, 840),}
        self.Time_windows_final_depots = {9: (480, 1440), 10: (480, 1440)}
        
        num_camions_0 = 4
        num_camions_NF1 = 4
        num_camions_NF2 = 4
        self.V_C0  = list(range(num_camions_0))                            # [0, 1]
        self.V_CNF1 = list(range(num_camions_0, num_camions_0 + num_camions_NF1))  # [2,3]
        self.V_CNF2 = list(range(num_camions_0 + num_camions_NF1, num_camions_0 + num_camions_NF1 + num_camions_NF2))  # [4,5]
        self.V_C = self.V_C0 + self.V_CNF1 + self.V_CNF2
        self.camion_capacity = 6
        
        self.model = gp.Model("CEVRPTW") # "Capacitated Electrical Vehicle Routing Problem with Time Windows"
        # VARIABILI DI DECISIONE
        self.arc_activation_variable_R = self.model.addVars(self.A, self.V_R, vtype=GRB.BINARY, name="xij_r")
        self.arc_activation_variable_C = self.model.addVars(self.A_C, self.V_C, vtype=GRB.BINARY, name="xij_c") 
        self.bundle_transportation_variable = self.model.addVars(self.A_Bundle, self.V_R, self.V_C, vtype=GRB.BINARY, name="zij_rc")   
        self.speed_selection_variable = self.model.addVars(((i, j, k, r) for (i, j, k) in self.street_arcs for r in self.V_R), vtype=GRB.BINARY, name="yijk_r")    
        self.Gamma = self.model.addVars(((i, j, t, r) for (i, j, t) in self.arcs_metro_list for r in self.V_R), vtype=GRB.BINARY, name="Gammaijt_r")
        
        # Now, the remaining continuous variables
        self.freight_flow = self.model.addVars([(i, j, r) for (i, j, r) in self.arc_activation_variable_R.keys()],vtype=GRB.CONTINUOUS, lb=0, ub=self.robot_capacity, name="ff")
        self.battery_level = self.model.addVars(self.nodes, self.V_R, vtype=GRB.CONTINUOUS, lb=0, ub=self.battery_capacity, name="chob")
        self.arrival_time_r = self.model.addVars(self.nodes, self.V_R, vtype=GRB.CONTINUOUS, name="arrival_r") 
        self.arrival_time_truck = self.model.addVars(self.init_depot + self.N_T + self.N_C + self.N_F, self.V_C, vtype=GRB.CONTINUOUS, name="arrival_c")
        self.departure_time_r = self.model.addVars(self.nodes, self.V_R, vtype=GRB.CONTINUOUS, name="departure")
        self.departure_time_truck = self.model.addVars(self.init_depot + self.N_T + self.N_C + self.N_F, self.V_C, vtype=GRB.CONTINUOUS, name="departure_c")
        self.waiting_time_r = self.model.addVars(self.nodes, self.V_R, vtype=GRB.CONTINUOUS, lb=0, name="waiting")  
        
    def build_constraints(self):
        # === SYMMETRY-BREAKING AMONG ROBOTS-CUT ===
        for r in range(1, len(self.V_R)):
            used_r = gp.quicksum(
                self.arc_activation_variable_R[0, j, r]
                for j in self.nodes
                if (0, j, r) in self.arc_activation_variable_R)
            used_prev = gp.quicksum(
                self.arc_activation_variable_R[0, j, r-1]
                for j in self.nodes
                if (0, j, r-1) in self.arc_activation_variable_R)
            # Robot r can only be used if robot r-1 is also used
            self.model.addConstr(used_prev >= used_r, name=f"robot_symmetry_{r}")
            
    # --- VINCOLI DI FLUSSO NELLA RETE
        #(1...) I robot possono uscire al massimo una volta oppure non uscire
        for r in self.V_R:
            self.model.addConstr(gp.quicksum(self.arc_activation_variable_R[0, j, r] for j in self.nodes if (0, j, r) in self.arc_activation_variable_R) <= 1, name=f"robot_departure_from_depot{r}")
             
        #(2...) Uscita da zero-entrata a NF
        for r in self.V_R:
            self.model.addConstr(gp.quicksum(self.arc_activation_variable_R[0, j, r] for j in list(self.N_T) if (0, j, r) in self.arc_activation_variable_R) == gp.quicksum(self.arc_activation_variable_R[i, f, r] for i in list(self.N_C) + list(self.N_T) for f in self.N_F if (i, f, r) in self.arc_activation_variable_R), name=f"flow_from_depot_equals_to_final_r{r}")
        
        #(3...)Bilancio del flusso nodi stations y customers
        for r in self.V_R:
            for i in self.nodes:
                if i != 0 and i not in self.N_F:
                    self.model.addConstr(gp.quicksum(self.arc_activation_variable_R[i, j, r] for j in self.nodes if j != i if (i, j, r) in self.arc_activation_variable_R) - gp.quicksum(self.arc_activation_variable_R[j, i, r] for j in self.nodes if j != i and (j, i, r) in self.arc_activation_variable_R) == 0, name=f"flow_conservation_node_{i}_robot_{r}")    
        
        #(4...) No split-delivery
        for j in self.N_C:
            self.model.addConstr(gp.quicksum(self.arc_activation_variable_R[i, j, r] for r in self.V_R for i in self.nodes if i != j and i not in self.N_F and (i, j, r) in self.arc_activation_variable_R) == 1, name=f"customer_visited_once_N_C{j}")    
                
        # (5...)Selezione di una sola velocità   
        for (i, j) in self.A_RS:
            if (i, j) not in self.A_Bundle:
                for r in self.V_R:
                    self.model.addConstr(gp.quicksum(self.speed_selection_variable[i, j, k, r] for k in self.K) == self.arc_activation_variable_R[i, j, r], name=f"select_one_speed_{i}_{j}_r{r}")
        
        # (***) TAGLIO STATION STATION
        for r in self.V_R:
            for i in list(self.N_T):
                self.model.addConstr(gp.quicksum(self.arc_activation_variable_R[i, j, r] for (si, j) in self.A_P if si == i) >= self.arc_activation_variable_R[0, i, r], name=f"must_take_metro_if_from_depot_s{i}_r{r}")
       
######################################################################################################################
    
    # --- VEHICLE CAPACITY CONSTRAINTS
        #(6...)Conservazione del flusso di merce
        for r in self.V_R: 
            for j in self.nodes:
                lhs = (gp.quicksum(self.freight_flow[i, j, r] for i in self.nodes if i != j and i not in self.N_F and (i, j, r) in self.freight_flow) - gp.quicksum(self.freight_flow[j, g, r] for g in self.nodes if g != j and g != 0 and (j, g, r) in self.freight_flow))
                if j in self.N_C:  
                    self.model.addConstr(lhs == self.demands[j] * gp.quicksum(self.arc_activation_variable_R[i, j, r] for i in self.nodes if i != j and i not in self.N_F and (i, j, r) in self.arc_activation_variable_R), name=f"flow_load_customer_j{j}_r{r}")
                elif (j in self.stations_and_copies) or (j in self.N_F): 
                    self.model.addConstr(lhs == 0, name=f"flow_load_transit_j{j}_r{r}")
                    
        #(7...)Capacità dei robot
        for r in self.V_R: 
            for i in self.nodes:
                if i not in self.N_F: 
                    for j in self.nodes:
                        if j != 0 and i != j and (i, j, r) in self.arc_activation_variable_R:   
                            self.model.addConstr(self.freight_flow[i, j, r] <= self.robot_capacity * self.arc_activation_variable_R[i, j, r], name=f"capacity_{i}_j{j}_r{r}")
        
        #(8...)Esce dal depot esattamente quello che si andrà a distribuire in ogni rotta
        for r in self.V_R:
            self.model.addConstr(gp.quicksum(self.freight_flow[0, j, r] for j in self.nodes if j != 0 and (0, j, r) in self.freight_flow) == gp.quicksum(self.demands[j] * self.arc_activation_variable_R[i, j, r] for j in self.N_C for i in self.nodes if i != j and (i, j, r) in self.arc_activation_variable_R), name=f"flow_from_depot_load_r{r}")

######################################################################################################################

        #BATTERY CONSTRAINTS for the robots
        #(9...) Carica piena nel depot 
        for r in self.V_R:
            self.model.addConstr(self.battery_level[0, r] == self.battery_capacity, name=f"depot_charge_{r}")
     
        # (10...) Evoluzione della batteria negli archi di strada (solo in funzione di x_ijr)
        for (i, j) in self.A_RS:
            for r in self.V_R:
                self.model.addConstr(self.battery_level[j, r] <= self.battery_level[i, r] - gp.quicksum(self.battery_consumption[i, j, k] * self.speed_selection_variable[i, j, k, r] for k in self.K)
                    + self.M * (1 - self.arc_activation_variable_R[i, j, r]), name=f"bat_street_up_{i}_{j}_r{r}")
                self.model.addConstr(self.battery_level[j, r] >= self.battery_level[i, r]
                    - gp.quicksum(self.battery_consumption[i, j, k] * self.speed_selection_variable[i, j, k, r] for k in self.K) - self.M * (1 - self.arc_activation_variable_R[i, j, r]), name=f"bat_street_down_{i}_{j}_r{r}")
        
        #(11...) Evoluzione negli archi di metro, non c'è consumo, la batteria si conserva 
        for (i, j, t) in self.arcs_metro_list:
            for r in self.V_R:
                self.model.addConstr(self.battery_level[j, r] <= self.battery_level[i, r] + self.M * (1 - self.Gamma[i, j, t, r]), name=f"bat_metro_up_{i}_{j}_{t}_r{r}")
                self.model.addConstr(self.battery_level[j, r] >= self.battery_level[i, r] - self.M * (1 - self.Gamma[i, j, t, r]), name=f"bat_metro_down_{i}_{j}_{t}_r{r}")

######################################################################################################################

        #TIME MANAGEMENT
        # GESTIONE DEL DEPARTURE TIME
        #(12...) Departure from depot zero after its opening
        for r in self.V_R:  
            self.model.addConstr(self.departure_time_r[0, r] >= self.init_depot_TW[0], name=f"departure_from_depot0_v{r}")
        
        #(13...) Scelta di un solo viaggio per partire dalle stazioni
        for (i, j) in self.A_P:
            for r in self.V_R:
                self.model.addConstr(gp.quicksum(self.Gamma[i, j, t, r] for (ii, jj, t) in self.arcs_metro_list if ii == i and jj == j) == self.arc_activation_variable_R[i, j, r], name=f"link_x_Gamma_{i}_{j}_r{r}")
                 
        #(14...) Il departure e la partenza del viaggio scelto (in NT+NTP)
        for i in self.stations_and_copies:
            for r in self.V_R:
                use_metro_in_i_j = gp.quicksum(self.Gamma[i, j, t, r] for (ii, j, t) in self.arcs_metro_list if ii == i)
                metro_departure = gp.quicksum(self.dep_time_i[(i, j, t)] * self.Gamma[i, j, t, r] for (ii, j, t) in self.arcs_metro_list if ii == i)
                self.model.addConstr(self.departure_time_r[i, r] >= metro_departure - self.M * (1 - use_metro_in_i_j), name=f"dep_metro_low_{i}_r{r}")
                self.model.addConstr(self.departure_time_r[i, r] <= metro_departure + self.M * (1 - use_metro_in_i_j), name=f"dep_metro_up_{i}_r{r}")
                
        #(15...) departure de clientes >= arrival + waiting + service - M*(1-SUMA Xjir)      
        for r in self.V_R:
            for i in self.N_C:
                arcs_enter = gp.quicksum(self.arc_activation_variable_R[j, i, r] for j in self.nodes if (j, i, r) in self.arc_activation_variable_R)
                self.model.addConstr(self.departure_time_r[i, r] >= self.arrival_time_r[i, r] + self.waiting_time_r[i, r] + self.service_time[i] - self.M * (1 - arcs_enter))
                self.model.addConstr(self.departure_time_r[i, r] <= self.arrival_time_r[i, r] + self.waiting_time_r[i, r] + self.service_time[i] + self.M * (1 - arcs_enter))

        #(16...) departure de "NT+NTP": departure >= arrival + waiting  (sin service_time)
        for r in self.V_R:
            for i in self.stations_and_copies:
                arcs_enter = gp.quicksum(self.arc_activation_variable_R[j, i, r] for j in self.nodes if (j, i, r) in self.arc_activation_variable_R)
                self.model.addConstr(self.departure_time_r[i, r] >= self.arrival_time_r[i, r] + self.waiting_time_r[i, r] - self.M * (1 - arcs_enter))
                self.model.addConstr(self.departure_time_r[i, r] <= self.arrival_time_r[i, r] + self.waiting_time_r[i, r] + self.M * (1 - arcs_enter))      
        
        # (17 and 18...) ARRIVAL - Evoluzione del tempo negli archi di strada ajr>=dir+SUMA(TVSijk*y_ijkr)-M*(1-xijr)
        for (i, j) in self.A_RS:
            for r in self.V_R:
                if (i, j) in self.A_Bundle:
                    relax_expression = (1 - self.arc_activation_variable_R[i, j, r]  + gp.quicksum(self.bundle_transportation_variable[i, j, r, c] for c in self.V_C))
                else:
                    relax_expression = 1 - self.arc_activation_variable_R[i, j, r]
                self.model.addConstr(self.arrival_time_r[j, r] >= self.departure_time_r[i, r] 
                    + gp.quicksum(self.time_street[i, j, k] * self.speed_selection_variable[i, j, k, r] for k in self.K)
                    - self.M * relax_expression, name=f"time_street_down_{i}_{j}_r{r}")
                self.model.addConstr(self.arrival_time_r[j, r] <= self.departure_time_r[i, r]
                    + gp.quicksum(self.time_street[i, j, k] * self.speed_selection_variable[i, j, k, r] for k in self.K)
                    + self.M * relax_expression, name=f"time_street_up_{i}_{j}_r{r}")

        #(19...) ARRIVAL - Evoluzione del tempo negli archi di metro   ajr>=dir+TMij-M*(1-xijr)       
        for (i, j, t) in self.arcs_metro_list:
            for r in self.V_R: 
                self.model.addConstr(self.arrival_time_r[j, r] >= self.departure_time_r[i, r] + self.TTM[(i, j, t)] - self.M * (1 - self.Gamma[i, j, t, r]), name=f"time_metro_down_{i}_{j}_{t}_r{r}")
                self.model.addConstr(self.arrival_time_r[j, r] <= self.departure_time_r[i, r] + self.TTM[(i, j, t)] + self.M * (1 - self.Gamma[i, j, t, r]), name=f"time_metro_up_{i}_{j}_{t}_r{r}")
    
######################################################################################################################
        
        # WAITING TIME MANAGEMENT
        #(20...) Waiting en los customers
        for r in self.V_R:       
            for i in self.N_C:   
                self.model.addConstr(self.waiting_time_r[i, r] >= self.Time_windows_customers[i][0] - self.arrival_time_r[i, r], name=f"waiting_time_customer_i{i}_r{r}")
        
        #(21...) Waiting en las estaciones: solo si se usa algún viaje de metro desde i
        for r in self.V_R:
            for i in self.stations_and_copies:
                use_metro_in_i_j = gp.quicksum(self.Gamma[i, j, t, r] for (ii, j, t) in self.arcs_metro_list if ii == i)
                metro_departure = gp.quicksum(self.dep_time_i[(i, j, t)] * self.Gamma[i, j, t, r] for (ii, j, t) in self.arcs_metro_list if ii == i)
                self.model.addConstr(self.waiting_time_r[i, r] >= metro_departure - self.arrival_time_r[i, r] - self.M * (1 - use_metro_in_i_j), name=f"waiting_station_low_{i}_r{r}")
                self.model.addConstr(self.waiting_time_r[i, r] <= metro_departure - self.arrival_time_r[i, r] + self.M * (1 - use_metro_in_i_j), name=f"waiting_station_up_{i}_r{r}")

###################################################################################################################### 
        
        # EARLY ARRIVAL MANAGEMENT  
        #(22...) Robots deben llegar a los nodos finales N_F y a los customers antes de que estos cierren:  
        for r in self.V_R:  
            for i in self.N_F: 
                self.model.addConstr(self.arrival_time_r[i, r] <= self.Time_windows_final_depots[i][1] - self.service_time[i], name=f"time_window_robot_i{i}_v{r}")
            for i in self.N_C: 
                self.model.addConstr(self.arrival_time_r[i, r] <= self.Time_windows_customers[i][1] - self.service_time[i], name=f"time_robot_i{i}_v{r}")

######################################################################################################################
    
        # ADDING THE DYNAMICS OF THE TRUCKS
        #(23...) Truck exit from the depotiniziale
        for c in self.V_C0:
            self.model.addConstr(gp.quicksum(self.arc_activation_variable_C[0, i, c] for i in self.N_T) <= 1, name=f"truck_departure_from_depot{c}")

        #(24...) Exit from the NF1 and NF2
        # Para los depots finales, los camiones pueden salir a rescatar los robots a las estaciones o a los customers
        for c in self.V_CNF1:
            self.model.addConstr(gp.quicksum(self.arc_activation_variable_C[f, i, c] for (f, i) in self.A_C if (f in self.NF1) and (i in list(self.N_C) + list(self.N_T))) <= 1, name=f"truck_from_final_depot1_c{c}")
        for c in self.V_CNF2:
            self.model.addConstr(gp.quicksum(self.arc_activation_variable_C[f, i, c] for (f, i) in self.A_C if (f in self.NF2) and (i in list(self.N_C) + list(self.N_T))) <= 1, name=f"truck_from_final_depot2_c{c}")
        
        #(25...) Truck balance at 0
        for c in self.V_C0:
            self.model.addConstr(gp.quicksum(self.arc_activation_variable_C[0, i, c] for i in list(self.N_T)) == gp.quicksum(self.arc_activation_variable_C[i, 0, c] for i in list(self.N_T)), name=f"flow_from_depot_equals_to_final_c{c}")

        # (26...) Truck flow balance in bundle at stations (robots at 0) and for those i NF at stations and customers
        for c in self.V_C0:
            for i in self.N_T:
                if (0, i) in self.A_Bundle and (i, 0) in self.A_C:
                    self.model.addConstr(gp.quicksum(self.bundle_transportation_variable[0, i, r, c] for r in self.V_R) <= self.arc_activation_variable_C[i, 0, c], name=f"return_same_station_i{i}_truck_c{c}")
        for c in self.V_CNF1:
            for j in self.N_C + self.N_T:
                if (j, 9) in self.A_Bundle and (9, j) in self.A_C:
                    self.model.addConstr(gp.quicksum(self.bundle_transportation_variable[j, 9, r, c] for r in self.V_R) <= self.arc_activation_variable_C[9, j, c], name=f"return_from_NF1_same_customer_j{j}_c{c}")
        for c in self.V_CNF2:
            for j in self.N_C + self.N_T:
                if (j, 10) in self.A_Bundle and (10, j) in self.A_C:
                    self.model.addConstr(gp.quicksum(self.bundle_transportation_variable[j, 10, r, c] for r in self.V_R) <= self.arc_activation_variable_C[10, j, c], name=f"return_from_NF2_same_customer_j{j}_c{c}")
                    
        #(27...) Managing speed in bundle
        for r in self.V_R:
            for (i,j) in self.A_Bundle:
                self.model.addConstr(gp.quicksum(self.speed_selection_variable[i, j, k, r] for k in self.K) == self.arc_activation_variable_R[i, j, r]
                    - gp.quicksum(self.bundle_transportation_variable[i, j, r, c] for c in self.V_C), name=f"bundle_i{i}_j{j}_r{r}")
          
        # Vinculo de allacio non può esserci attivazione del camion senza l’attivazione del robot che deve trasportare
        #(28...) Zijrc<=Xijr
        for (i, j, r, c) in self.bundle_transportation_variable.keys():
            if (i, j, r) in self.arc_activation_variable_R:
                self.model.addConstr(self.bundle_transportation_variable[i, j, r, c] <= self.arc_activation_variable_R[i, j, r], name=f"z_leq_x_i{i}_j{j}_r{r}_c{c}")
        
        #(29...) Zijrc<=Xijc
        for (i, j, r, c) in self.bundle_transportation_variable.keys():
            if (i, j, c) in self.arc_activation_variable_C:    
                self.model.addConstr(self.bundle_transportation_variable[i, j, r, c] <= self.arc_activation_variable_C[i, j, c], name=f"z_leq_xC_i{i}_j{j}_r{r}_c{c}")    
        
        #(30...) Truck capacity
        for (i, j) in self.A_C:
            for c in self.V_C:
                self.model.addConstr(gp.quicksum(self.bundle_transportation_variable[i, j, r, c]
                        for r in self.V_R if (i, j, r, c) in self.bundle_transportation_variable)
                    <= self.camion_capacity * self.arc_activation_variable_C[i, j, c],
                    name=f"truck_robot_cap_c{c}_i{i}_j{j}")
                
        #(31...) Managing truck departures from 0, NF1 and NF2
        for c in self.V_C0:  
            self.model.addConstr(self.departure_time_truck[0, c] >= self.init_depot_TW[0], name=f"departure_from_depot0_v{c}")
        for c in self.V_CNF1:
            for f in self.NF1:  
                self.model.addConstr(self.departure_time_truck[f, c] >= self.Time_windows_final_depots[f][0], name=f"departure_from_depotNF1c{c}_f{f}")           
        for c in self.V_CNF2:
            for f in self.NF2:  
                self.model.addConstr(self.departure_time_truck[f, c] >= self.Time_windows_final_depots[f][0], name=f"departure_from_depotNF2c{c}_f{f}")           
        
        #(32...) Managing  truck arrivals   ajc>=dic+TCij-M*(1-xijc)     
        for (i, j) in self.A_C:
            for c in self.V_C: 
                self.model.addConstr(self.arrival_time_truck[j, c] >= self.departure_time_truck[i, c] + self.time_camions[(i, j)] - self.M * (1 - self.arc_activation_variable_C[i, j, c]), name=f"time_truck_down_{i}_{j}_c{c}")
                self.model.addConstr(self.arrival_time_truck[j, c] <= self.departure_time_truck[i, c] + self.time_camions[(i, j)] + self.M * (1 - self.arc_activation_variable_C[i, j, c]), name=f"time_truck_up_{i}_{j}_c{c}")
        
        # (33...) Time management trucks at the visited nodes
        # departure_time_truck[i,c] = arrival_time_truck[i,c] se il camion entra in i
        all_truck_nodes = self.init_depot + self.N_T + self.N_C + self.N_F
        for c in self.V_C:
            if c in self.V_C0:
                depot_home = 0
            elif c in self.V_CNF1:
                depot_home = 9
            else:
                depot_home = 10
            for i in all_truck_nodes:
                if i == depot_home:
                    continue
                # il camion entra davvero ad i????
                incoming = gp.quicksum(self.arc_activation_variable_C[j, i, c] for j in all_truck_nodes if j != i and (j, i) in self.A_C and (j, i, c) in self.arc_activation_variable_C)
                # se incoming = 1 → dep_i = arr_i
                self.model.addConstr(self.departure_time_truck[i, c] >= self.arrival_time_truck[i, c] - self.M * (1 - incoming), name=f"truck_time_consistency_low_i{i}_c{c}")
                self.model.addConstr(self.departure_time_truck[i, c] <= self.arrival_time_truck[i, c] + self.M * (1 - incoming), name=f"truck_time_consistency_up_i{i}_c{c}")
        
        #(34...) Managing truck and robot bundle departures
        for (i, j) in self.A_Bundle:
            for r in self.V_R:
                for c in self.V_C:
                    self.model.addConstr(self.departure_time_r[i, r] >= self.departure_time_truck[i, c]
                        - self.M * (1 - self.bundle_transportation_variable[i, j, r, c]),
                        name=f"time_truck_bundle_depart_down_i{i}_j{j}_r{r}_c{c}")
                    self.model.addConstr(self.departure_time_r[i, r] <= self.departure_time_truck[i, c]
                        + self.M * (1 - self.bundle_transportation_variable[i, j, r, c]),
                        name=f"time_truck_bundle_depart_up_i{i}_j{j}_r{r}_c{c}")
        
        #(35...) Managing  truck and robot bundle arrivals
        for (i, j) in self.A_Bundle:
            for r in self.V_R:
                for c in self.V_C:
                    self.model.addConstr(self.arrival_time_r[j, r] >= self.arrival_time_truck[j, c]
                        - self.M * (1 - self.bundle_transportation_variable[i, j, r, c]),
                        name=f"time_truck_bundle_arrive_down_i{i}_j{j}_r{r}_c{c}")
                    self.model.addConstr(self.arrival_time_r[j, r] <= self.arrival_time_truck[j, c]
                        + self.M * (1 - self.bundle_transportation_variable[i, j, r, c]),
                        name=f"time_truck_bundle_arrive_up_i{i}_j{j}_r{r}_c{c}")
        
        #(36...) --- DOMAIN of the TRUCKS: each truck departs and returns to its depot
        for c in self.V_C0:
            for (i, j) in self.A_C:
                if 0 not in (i, j):
                    self.model.addConstr(self.arc_activation_variable_C[i, j, c] == 0, name=f"forbid_arc_truck0_c{c}_i{i}_j{j}")
        for c in self.V_CNF1:
            for (i, j) in self.A_C:
                if 9 not in (i, j):
                    self.model.addConstr(self.arc_activation_variable_C[i, j, c] == 0, name=f"forbid_arc_truck9_c{c}_i{i}_j{j}")
        for c in self.V_CNF2:
            for (i, j) in self.A_C:
                if 10 not in (i, j):
                    self.model.addConstr(self.arc_activation_variable_C[i, j, c] == 0, name=f"forbid_arc_truck10_c{c}_i{i}_j{j}")        
        
        #(37...) --- I camion dal depot 0 portano, gli altri vanno a prendere robot
        for c in self.V_C0:
            for i in self.N_T:
                if (0, i) in self.A_Bundle and (0, i) in self.A_C:
                    self.model.addConstr(
                        self.arc_activation_variable_C[0, i, c] <= gp.quicksum(self.bundle_transportation_variable[0, i, r, c] for r in self.V_R if (0, i, r, c) in self.bundle_transportation_variable), name=f"arc0i_needs_bundle_c{c}_i{i}")

######################################################################################################################       

        #(38...) --- E-constraint
        f_value = 33.49
        waiting_total = gp.quicksum(self.waiting_time_r[i, r] for (i, r) in self.waiting_time_r.keys())

        term_street_time = gp.quicksum(
            self.time_street[(i, j, k)] * self.speed_selection_variable[i, j, k, r]
            for (i, j, k, r) in self.speed_selection_variable.keys()
        )
        self.model.addConstr(waiting_total + term_street_time <= (self.epsilon + 1)*f_value, name="EConstraint_Waiting")
        
######################################################################################################################       
            
    def set_objective(self):
        # Costos (ajústalos a gusto)
        C_DIST_ROBOT = 1.0      # costo por unidad de distancia recorrida por los robots
        C_FIX_TRUCK  = 1000.0   # costo fijo por activar un camión
        CDC = 1.0
        # --- 1) Costo por distancia recorrida por los robots en calle ---

        # Distancias solo en los arcos de calle
        distances_on_street = {
            (i, j): d
            for (i, j), d in self.distances_robots_streets.items()
            if (i, j) in self.A_RS}
        term_robot_distance = gp.quicksum(
            distances_on_street[(i, j)] * self.speed_selection_variable[i, j, k, r]
            for (i, j, k) in self.street_arcs
            for r in self.V_R)
        truck_activation_expr = (
            gp.quicksum(
                self.arc_activation_variable_C[0, j, c]
                for c in self.V_C0
                for j in self.N_T + self.N_C + self.N_F
                if (0, j, c) in self.arc_activation_variable_C)
            + gp.quicksum(
                self.arc_activation_variable_C[9, j, c]
                for c in self.V_CNF1
                for j in self.N_T + self.N_C + self.N_F
                if (9, j, c) in self.arc_activation_variable_C)
            + gp.quicksum(
                self.arc_activation_variable_C[10, j, c]
                for c in self.V_CNF2
                for j in self.N_T + self.N_C + self.N_F
                if (10, j, c) in self.arc_activation_variable_C))
        term_truck_distance = gp.quicksum(
            self.true_distances_camions_dict[(i, j)] * self.arc_activation_variable_C[i, j, c]
            for (i, j) in self.A_C
            if (i, j) in self.true_distances_camions_dict
            for c in self.V_C)
        
            # --- 3) Función objetivo final: costos operativos ---
        self.model.setObjective(
            C_DIST_ROBOT * term_robot_distance
            + C_FIX_TRUCK  * truck_activation_expr + CDC * term_truck_distance,
            GRB.MINIMIZE)
        
    # NOW, I build the skeleton of the model and order its execution
    def build(self):
        self.build_constraints()
        self.set_objective()
        self.model.update()

    def solve(self):
        self.model.setParam('TimeLimit', 1800)
        self.model.Params.MIPFocus = 2   # prioriza mejorar el bound
        self.model.optimize()
    
    # THIS TINY METHOD IS VITAL, because one needs to convert the time from a continuous value to a time format     
    def format_time(self, minutes):
        hours = int(minutes // 60)
        minutes_remainder = int(minutes % 60)
        seconds_remainder = int((minutes - int(minutes)) * 60)
        return f"{hours:02d}:{minutes_remainder:02d}:{seconds_remainder:02d}"

    def report(self):
        print("Optimization complete. Status:", self.model.status)

        # 1) Caso realmente infactible
        if self.model.status == GRB.INFEASIBLE:
            try:
                print("MODEL IS INFEASIBLE. Calculating IIS...")
                self.model.computeIIS()
                self.model.write("modelo.ilp")
                print("IIS report generated: modelo.ilp")
            except Exception as e:
                print("Error computing IIS:", e)
            return  # aquí no hay solución que reportar

        # 2) Si no hay ninguna solución factible
        if self.model.SolCount == 0:
            print("No feasible solution found (SolCount = 0).")
            return

        # 3) Hay al menos una solución factible: reportar la mejor
        if self.model.status == GRB.OPTIMAL:
            print("\nYUPIII!!! OPTIMAL SOLUTION FOUND")
        elif self.model.status == GRB.TIME_LIMIT:
            print("\nTime limit reached. Reporting best incumbent solution.")
            try:
                print(f"Current MIP gap: {100 * self.model.MIPGap:.2f}%")
            except:
                pass
        else:
            print(f"\nSolver finished with status {self.model.status}.")
            print("Reporting best incumbent solution found.")
            
        #########################################################################    
        print(f"OBJECTIVE_FUNCTION_VALUE(cost): {self.model.objVal:.2f}\n")   
        ########################################################################     
        try:
            print(f"GAP final: {100 * self.model.MIPGap:.2f}%")
        except Exception:
            pass
        print(f"Tiempo de cómputo: {self.model.Runtime:.2f} s")
 
        waiting_total = sum(
            self.waiting_time_r[i, r].X
            for (i, r) in self.waiting_time_r.keys()
        )
        print(f"Tiempo total de espera: {waiting_total:.2f} min")
        
        Waiting_and_travel_time_r = sum(self.waiting_time_r[i, r].X for (i, r) in self.waiting_time_r.keys()) + sum(self.time_street[(i, j, k)] * self.speed_selection_variable[i, j, k, r].X for (i, j, k, r) in self.speed_selection_variable.keys())
        print(f"Tiempo_total_espera + travel_on_street: {Waiting_and_travel_time_r:.2f} min")

        total_battery_consumption = 0.0
        for (i, j, k, r) in self.speed_selection_variable.keys():
            if self.speed_selection_variable[i, j, k, r].X > 0.5:
                total_battery_consumption += self.battery_consumption[(i, j, k)]
        print(f"Consumo batería total en calle: {total_battery_consumption:.2f} mAh")


        trucks_used = set()
        for (i, j, c) in self.arc_activation_variable_C.keys():
            if self.arc_activation_variable_C[i, j, c].X > 0.5:
                trucks_used.add(c)
        truck_number = len(trucks_used)
        print(f"Número de camiones utilizados: {truck_number}\n")
        # ======================================================
        
        print("\nArchi attivati per ogni robot:")      
        activated_arcs_by_robot = {r: [] for r in self.V_R}
        for (i, j, r) in self.arc_activation_variable_R.keys():
            if self.arc_activation_variable_R[i, j, r].x > 0.5:
                activated_arcs_by_robot[r].append((i, j))
        for r in self.V_R:
            if not activated_arcs_by_robot[r]:
                print(f"Robot {r} remains unused at the depot.")
            else:
                print(f"\nRobot {r} was used and drives along these arcs:")
                for (i, j) in activated_arcs_by_robot[r]:
                    print(f"  ({i} → {j})")
            
        ######################################################################## 

        print("\nVariabili di velocità:") 
        selected_speeds = []
        for (i, j, k, r) in self.speed_selection_variable.keys():
            if self.speed_selection_variable[i, j, k, r].x > 0.5:
                selected_speeds.append((i, j, k, r))
            
        for (i, j, k, r) in selected_speeds:
            print(f"Robot {r}: travels ({i}-{j}) with speed {k}")
                
        ######################################################################## 
        
        print("\nVariabili di viaggio in metro:")            
        selected_trips = []
        for (i, j, t, r) in self.Gamma.keys():
            if self.Gamma[i, j, t, r].x > 0.5:
                selected_trips.append((r, i, j, t))
            
        for (r, i, j, t) in selected_trips:
            line_name = self.line_arc[(i, j, t)]
            dep = self.dep_time_i[(i, j, t)]   
            print(f"Robot {r}: {i}-{j} using trip {t} of line {line_name} (dep = {self.format_time(dep)})")
                
        ######################################################################## 
        depot = 0
        print("\nDEPARTURE TIME FROM THE INITIAL DEPOT (hh:mm:ss):")

        for r in self.V_R:
            used = False
            for j in self.nodes:  # o la lista que tenga los nodos destino posibles
                if (depot, j, r) in self.arc_activation_variable_R:
                    if self.arc_activation_variable_R[depot, j, r].x > 0.5:   # se usó
                        used = True
                        break

            if used and (depot, r) in self.departure_time_r:
                dep_time = self.departure_time_r[depot, r].x  
                dep_str = self.format_time(dep_time)
                dep_raw_rounded = round(dep_time, 2)
                print(f"Robot {r} DEPARTURE raw_value: {dep_raw_rounded}, TIME_{dep_str}")

        ########################################################################
        
        print("\nSCHEDULED_ARRIVALS_AND_DEPARTURES (formato hh:mm:ss):")
        for i in self.nodes:
            for r in self.V_R:
                entered_the_node = sum(
                    self.arc_activation_variable_R[j, i, r].x
                    for j in self.nodes
                    if j != i and (j, i, r) in self.arc_activation_variable_R)
                if entered_the_node > 0.5:
                    arr = self.arrival_time_r[i, r].x
                    arr_rounded = round(arr, 2)  
                    wait = self.waiting_time_r[i, r].x
                    wait_rounded = round(wait, 2) 
                    dep = self.departure_time_r[i, r].x
                    dep_rounded = round(dep, 2)
                    arr_str = self.format_time(arr)
                    dep_str = self.format_time(dep)
                    print(
                        f"Robot {r}: arr_{i}: ({arr_rounded}), TIME_{arr_str},"
                        f"waiting_node_{i}: {wait_rounded:.2f}min, "
                        f"dep_: ({dep_rounded}), TIME_{dep_str},")
                        
        print("\nARC TRAVEL TIMES (TV = arr_j - dep_i):")
        for r in self.V_R:
            for i in self.nodes:
                for j in self.nodes:
                    if i != j and (i, j, r) in self.arc_activation_variable_R:
                        if self.arc_activation_variable_R[i, j, r].x > 0.5:
                            dep = self.departure_time_r[i, r].x
                            arr = self.arrival_time_r[j, r].x
                            TV = round(arr - dep, 3)
                            print(f"Robot {r}, arc ({i} to {j}): "
                                f"TV = {TV}")
           
        depot0 = 0                  # nodo del depósito inicial del robot
        final_nodes = list(self.NF1) + list(self.NF2)   # o self.N_F si lo tienes así

        print("\nTOTAL WAITING TIME PER ROBOT (in minutes):")
        for r in self.V_R:
            robot_used = any(rr == r and self.arc_activation_variable_R[i, j, rr].X > 0.5 for (i, j, rr) in self.arc_activation_variable_R.keys())
            if not robot_used:
                continue 
            total_wait = sum(self.waiting_time_r[i, r].X for i in self.nodes if (i, r) in self.waiting_time_r)
            print(f"Robot {r}: {total_wait:.2f} min")
        
        print("\nROUTE DURATION PER ROBOT (arrival at final node - departure from depot 0):")

        for r in self.V_R:
            uses_robot = any((depot0, j, r) in self.arc_activation_variable_R and self.arc_activation_variable_R[depot0, j, r].X > 0.5 for j in self.nodes if j != depot0)
            if not uses_robot:
                continue
            start = self.departure_time_r[depot0, r].X
            reached_final_nodes = []
            for f in final_nodes:
                arrives_here = any((i, f, r) in self.arc_activation_variable_R and self.arc_activation_variable_R[i, f, r].X > 0.5 for i in self.nodes if i != f)
                if arrives_here:
                    reached_final_nodes.append(f)
            if not reached_final_nodes:
                continue
            end = max(self.arrival_time_r[f, r].X for f in reached_final_nodes)
            duration = end - start
            print(f"Robot {r}: duration = {round(duration, 3)} (end at nodes {reached_final_nodes})")

        ########################################################################
   
        print("\nFREIGHT AMOUNT LOADED ON EACH VEHICLE AND VISITED NODES:")

        for r in self.V_R:
            vehicle_used = any(
                (i, j, rr) in self.arc_activation_variable_R and self.arc_activation_variable_R[i, j, rr].x > 0.5
                for (i, j, rr) in self.arc_activation_variable_R.keys()
                if i == 0 and rr == r)
            if not vehicle_used:
                continue
            load_delivered = 0
            visited_nodes = []
            for i in self.N_C:
                visit_indicator = sum(
                    self.arc_activation_variable_R[m, i, r].x
                    for m in self.nodes
                    if m != i and (m, i, r) in self.arc_activation_variable_R)
                if visit_indicator > 0.5:
                    load_delivered += self.demands[i]
                    visited_nodes.append(i)
            print(f"Vehicle {r} must be loaded with: {load_delivered} (units)")
            print(f"Vehicle {r} must visit nodes: {visited_nodes}")
            
        ########################################################################
              
        print("\nBATTERY_LEVEL_AT_EACH_NODE:")
        for i in self.nodes:
            for r in self.V_R:
                incoming = sum(self.arc_activation_variable_R[j, i, r].x for j in self.nodes if j != i and (j, i, r) in self.arc_activation_variable_R)
                if incoming > 0.5:
                    print(f"Vehicle {r} (Node {i}): {self.battery_level[i, r].x:.2f} MAh")
 
        print("\nTRUCK ROUTES (ARCS ACTIVATED)")
        for c in self.V_C:
            used_arcs = [
                (i, j) for (i, j) in self.A_C
                if (i, j, c) in self.arc_activation_variable_C
                and self.arc_activation_variable_C[i, j, c].X > 0.5]
            if not used_arcs:
                continue  

            print(f"\nTruck {c} is ACTIVE on arc:")
            for (i, j) in used_arcs:
                print(f"  arc ({i} to {j})")
            
        print("\nTRUCK ACTIVATION (DEPARTURE FROM DEPOTS)")
        depot0 = 0 
        for c in self.V_C:
            dep_from_0 = [
                j for j in list(self.N_T) + list(self.N_C)
                if (depot0, j, c) in self.arc_activation_variable_C
                and self.arc_activation_variable_C[depot0, j, c].X > 0.5]
            dep_from_NF1 = [
                (f, j) for f in self.NF1 for j in list(self.N_T) + list(self.N_C)
                if (f, j, c) in self.arc_activation_variable_C
                and self.arc_activation_variable_C[f, j, c].X > 0.5]
            dep_from_NF2 = [
                (f, j) for f in self.NF2 for j in list(self.N_T) + list(self.N_C)
                if (f, j, c) in self.arc_activation_variable_C
                and self.arc_activation_variable_C[f, j, c].X > 0.5]
            if not dep_from_0 and not dep_from_NF1 and not dep_from_NF2:
                continue 
            print(f"\nTruck {c} activated from:")
            if dep_from_0:
                for j in dep_from_0:
                    print(f"  depot 0 to station: {j}")
            if dep_from_NF1:
                for (f, j) in dep_from_NF1:
                    print(f" depot NF1 {f} to {j}")
            if dep_from_NF2:
                for (f, j) in dep_from_NF2:
                    print(f"depot  NF2 {f} to {j}")
            
        print("\nROBOT–TRUCK BUNDLES")
        for r in self.V_R:
            for c in self.V_C:
                for (i, j) in self.A_Bundle:
                    if (i, j, r, c) in self.bundle_transportation_variable:
                        if self.bundle_transportation_variable[i, j, r, c].X > 0.5:
                            print(f"Robot {r} va con Truck {c} en arco ({i} -> {j})")
        
        print("\nTRUCK DEPARTURE TIMES FROM THEIR DEPOTS (hh:mm:ss)")

        for c in self.V_C:
            if c in self.V_C0:  # 1) home to truck c
                depot_home = 0
            elif c in self.V_CNF1:
                depot_home = 9
            else:
                depot_home = 10
            used = any(
                (depot_home, j, c) in self.arc_activation_variable_C
                and self.arc_activation_variable_C[depot_home, j, c].X > 0.5
                for j in list(self.N_T) + list(self.N_C)
                if j != depot_home and (depot_home, j) in self.A_C)
            if not used:
                continue 
            dep = self.departure_time_truck[depot_home, c].X
            dep_str = self.format_time(dep)
            print(f"Truck {c} departs from depot {depot_home} at t = {round(dep, 2)} ({dep_str})")
            
        print("\nTRUCK ARRIVAL TIMES AT VISITED NODES (hh:mm:ss)")

        all_truck_nodes = self.init_depot + self.N_T + self.N_C + self.N_F
        for c in self.V_C:
            # Comprobamos si el camión se usa en alguna parte
            used_somewhere = any(
                (i, j, c) in self.arc_activation_variable_C
                and self.arc_activation_variable_C[i, j, c].X > 0.5
                for (i, j) in self.A_C)
            if not used_somewhere:
                continue

            print(f"\nTruck {c} schedule:")

            for i in all_truck_nodes:
                # ¿Llega el camión c al nodo i?
                incoming = sum(
                    self.arc_activation_variable_C[j, i, c].X
                    for j in all_truck_nodes
                    if j != i and (j, i, c) in self.arc_activation_variable_C)
                if incoming > 0.5:
                    arr = self.arrival_time_truck[i, c].X
                    arr_str = self.format_time(arr)
                    print(f"  arrives at node {i} at t = {round(arr, 2)} ({arr_str})")
            
        ######################################################################## 

    
    