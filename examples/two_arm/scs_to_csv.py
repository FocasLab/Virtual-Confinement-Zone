import numpy as np

class scs_csv_converter():
    def __init__(self):
            '''
            Class for converting scs file to csv
            '''
            pass
    
    def read_csv(self, file_name, file_type):
        '''
        Reads the csv file line by line and extracts the required data
        depending on whether it is a scots.csv/target.csv/obstacle.csv
        
        State dimensions, State bounds, State quantisation,
        Input dimensions, Input Bounds,Input quantisation,
        Target set, Obstacles set and Input values c
        orresponding to the particular state values 
        can be extracted
        '''
        
        self.s_dim = None
        self.u_dim = None
        
        with open(file_name, "r") as scs_file:
            self.text = scs_file.readlines()
    
        if file_type == "scots": line = 5
        else: line = 3

        self.s_dim = int(self.text[line][:-1])
        line += 3
        
        self.s_eta = self.extract_values(line, self.s_dim)
        line += self.s_dim + 3
        
        self.s_lb = self.extract_values(line, self.s_dim)
        line += self.s_dim + 3
        
        self.s_ub = self.extract_values(line, self.s_dim)
        line += self.s_dim + 4

        if file_type == "scots":
            self.u_dim = int(self.text[line][:-1])
            line += 3

            self.u_eta = self.extract_values(line, self.u_dim)
            line += self.u_dim + 3
            
            self.u_lb = self.extract_values(line, self.u_dim)
            line += self.u_dim + 3
            
            self.u_ub = self.extract_values(line, self.u_dim)
            line += self.u_dim + 5
            
            self.u_shape = (((self.u_ub - self.u_lb) / self.u_eta) + 1).astype('int')
        
        self.line = line
        self.s_shape = (((self.s_ub - self.s_lb) / self.s_eta) + 1).astype('int')

    
    def extract_values(self, start_index, num_indices): 
        '''
        Extract values from texts into numpy arrays
        '''
        return np.array([float(self.text[start_index+i][:-1]) for i in range(num_indices)], dtype="float")
    
    
    def index_to_state(self, index):
        '''
        Converts index to state depending
        upon bounds and quantisation
        '''
        coords = np.zeros(self.s_dim)
        for dim in range(self.s_dim):
            coords[dim] = index % self.s_shape[dim]
            index //= self.s_shape[dim]
        return self.s_lb + coords * self.s_eta
    
    
    def index_to_input(self, index):
        '''
        Converts index to state depending
        upon bounds and quantisation
        '''
        coords = np.zeros(self.u_dim)
        for dim in range(self.u_dim):
            coords[dim] = index % self.u_shape[dim]
            index //= self.u_shape[dim]
        return self.u_lb + coords * self.u_eta

    
    def get_state_input_idx(self, curr_line):
        '''
        Extract input and state indices from text line of scs
        Input it not considered in target/obstacle set
        '''
        line_list = curr_line[:-1].split(' ')
        state_idx = int(line_list[0])
        if self.u_dim is not None: input_idx = np.max(np.array([int(i) for i in line_list[1:]], dtype='int'))
        # if self.u_dim is not None: input_idx = np.min(np.array([int(i) for i in line_list[1:]], dtype='int'))
        else: input_idx = None
        return state_idx, input_idx
            
            
    def write_csv(self, filename):
        '''
        Write the extracted values to csv file
        '''
        csv_file = open(filename, "w+")
        text = ','.join([f"table{i}" for i in range(self.s_dim)])
        if self.u_dim is not None: text += ',' + ','.join([f"table{i}" for i in range(self.u_dim)])
        csv_file.write(text + '\n')

        for curr_line in self.text[self.line:-1]:
            state_idx, input_idx = self.get_state_input_idx(curr_line)
            state = np.round(self.index_to_state(state_idx), 5)
            text = ','.join([str(i) for i in state])

            if self.u_dim is not None:
                ctrl_input = np.round(self.index_to_input(input_idx), 5)
                text += ',' + ','.join([str(i) for i in ctrl_input])

            csv_file.write(text + '\n')
        
if __name__ == "__main__":
    print("\nINITIATING........")

    scs_csv = scs_csv_converter()
    
    print("READING SCOTS!!!")
    scs_csv.read_csv("./controller.scs", "scots")
    scs_csv.write_csv("./controller.csv")
    print("DONE!!!")