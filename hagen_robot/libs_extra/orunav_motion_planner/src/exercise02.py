import gym
import numpy as np
from numpy.core.fromnumeric import shape

class Exercise03:
    def __init__(self, env, hidden=1, popsize=4):
        self.pvariance = 0.1     # variance of initial parameters
        self.ppvariance = 0.02   # variance of perturbations
        self.nhiddens = hidden       # number of internal neurons
        # the number of inputs and outputs depends on the problem
        # we assume that observations consist of vectors of continuous value
        # and that actions can be vectors of continuous values or discrete actions
        self.env = env
        self.ninputs = self.env.observation_space.shape[0]
        if (isinstance(self.env.action_space, gym.spaces.box.Box)):
            self.noutputs = self.env.action_space.shape[0]
        else:
            self.noutputs = self.env.action_space.n

        self.sigma = 0.02
        self.popsize = popsize

        self.W1_size = self.nhiddens*self.ninputs
        self.W2_size = self.noutputs*self.nhiddens
        self.b1_size = self.nhiddens
        self.b2_size = self.noutputs
            
        self.num_params = self.W1_size + self.W2_size +  self.b1_size + self.b2_size
        self.theta = np.random.randn(self.popsize, self.num_params)
        self.fitness = []

       
    def set_parameters(self, genotype):
        W1 = np.reshape(genotype[0: self.W1_size], (self.nhiddens, self.ninputs))  
        W2 = np.reshape(genotype[self.W1_size: self.W1_size+self.W2_size], (self.noutputs, self.nhiddens))  
        b1 = np.reshape(genotype[self.W1_size+self.W2_size : self.W1_size+self.W2_size+self.b1_size], (self.b1_size, 1))  
        b2 = np.reshape(genotype[self.W1_size+self.W2_size+self.b1_size : self.W1_size+self.W2_size+self.b1_size+self.b2_size], (self.b2_size, 1))
        return W1, W2, b1, b2  

    def network_update(self, observation, W1, W2, b1, b2):
        # observation = env.reset()
        # convert the observation array into a matrix with 1 column and ninputs rows
        # observation = observation.resize(ninputs, 1)
        # compute the netinput of the first layer of neurons
        Z1 = np.dot(W1, observation) + b1
        # compute the activation of the first layer of neurons with the tanh function
        A1 = np.tanh(Z1)
        # compute the netinput of the second layer of neurons
        Z2 = np.dot(W2, A1) + b2
        # compute the activation of the second layer of neurons with the tanh function
        A2 = np.tanh(Z2)
        # if the action is discrete
        return A2
    
    def evolve(self, episodes=20, steps=20, generations=10):
        for generation_i in range(generations):
            geno_index = 0
            self.fitness = []
            for genotype in self.theta:
                W1, W2, b1, b2 = self.set_parameters(genotype)
                rewards = 0
                for i_episode in range(episodes):
                        observation = env.reset()
                        for t in range(steps):
                            # env.render()
                            A2 = self.network_update(observation, W1, W2, b1, b2)
                            #  select the action that corresponds to the most activated unit
                            if (isinstance(env.action_space, gym.spaces.box.Box)):
                                action = A2
                            else:
                                action = np.argmax(A2)
                            observation, reward, done, info = env.step(action)
                            rewards += reward
                            # print(reward, rewards)
                            if done:
                                # print("Episode finished after {} timesteps".format(t+1))
                                break
                geno_index += 1
                self.fitness.append((geno_index, rewards))
            env.close()
            self.fitness = np.array(self.fitness)
            week_indicies = self.fitness[np.argsort(self.fitness, axis=0)[:,1]][:,0][0: int(self.popsize/2)]
            strong_indices = self.fitness[np.argsort(self.fitness, axis=0)[:,1]][:,0][ int(self.popsize/2): self.popsize]
            print("Fintess average: ", np.average(self.fitness[:,1]), "Best fitness: ", self.fitness[int(strong_indices[-1])-1][1])
            for week, strong in zip(week_indicies, strong_indices):
                noise = np.random.normal(0, self.sigma, self.num_params)
                self.theta[int(week)-1] = self.theta[int(strong)-1] + noise 


env = gym.make('CartPole-v0')
exe03 = Exercise03(env, hidden=1, popsize=20)
exe03.evolve(episodes=3, steps=20, generations=10)

